#!/usr/bin/env python3

import rospy
import math
import threading
from tf2_ros import Buffer, TransformListener, TransformException
import numpy as np
from autoware_msgs.msg import Lane, DetectedObjectArray, Waypoint
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Vector3Stamped
from shapely.geometry import LineString, Point, Polygon
from shapely import prepare, intersects, intersection
from tf2_geometry_msgs import do_transform_vector3
from scipy.interpolate import interp1d
from numpy.lib.recfunctions import unstructured_to_structured

class SimpleLocalPlanner:

    def __init__(self):

        # Parameters
        self.output_frame = rospy.get_param("~output_frame")
        self.local_path_length = rospy.get_param("~local_path_length")
        self.transform_timeout = rospy.get_param("~transform_timeout")
        self.braking_safety_distance_obstacle = rospy.get_param("~braking_safety_distance_obstacle")
        self.braking_safety_distance_goal = rospy.get_param("~braking_safety_distance_goal")
        self.braking_reaction_time = rospy.get_param("braking_reaction_time")
        self.stopping_lateral_distance = rospy.get_param("stopping_lateral_distance")
        self.current_pose_to_car_front = rospy.get_param("current_pose_to_car_front")
        self.default_deceleration = rospy.get_param("default_deceleration")

        # Variables
        self.lock = threading.Lock()
        self.global_path_linestring = None
        self.global_path_distances = None
        self.distance_to_velocity_interpolator = None
        self.current_speed = None
        self.current_position = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        # Publishers
        self.local_path_pub = rospy.Publisher('local_path', Lane, queue_size=1, tcp_nodelay=True)

        # Subscribers
        rospy.Subscriber('global_path', Lane, self.path_callback, queue_size=None, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/localization/current_velocity', TwistStamped, self.current_velocity_callback, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber('/detection/final_objects', DetectedObjectArray, self.detected_objects_callback, queue_size=1, buff_size=2**20, tcp_nodelay=True)


    def path_callback(self, msg):
        if len(msg.waypoints) == 0:
            global_path_linestring = None
            global_path_distances = None
            distance_to_velocity_interpolator = None
            rospy.loginfo("%s - Empty global path received", rospy.get_name())

        else:
            waypoints_xyz = np.array([(w.pose.pose.position.x, w.pose.pose.position.y, w.pose.pose.position.z) for w in msg.waypoints])
            # convert waypoints to shapely linestring
            global_path_linestring = LineString(waypoints_xyz)
            prepare(global_path_linestring)

            # calculate distances between points, use only xy, and insert 0 at start of distances array
            global_path_distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xyz[:,:2], axis=0)**2, axis=1)))
            global_path_distances = np.insert(global_path_distances, 0, 0)

            # extract velocity values at waypoints
            velocities = np.array([w.twist.twist.linear.x for w in msg.waypoints])
            # create interpolator
            distance_to_velocity_interpolator = interp1d(global_path_distances, velocities, kind='linear', bounds_error=False, fill_value=0.0)

            rospy.loginfo("%s - Global path received with %i waypoints", rospy.get_name(), len(msg.waypoints))

        with self.lock:
            self.global_path_linestring = global_path_linestring
            self.global_path_distances = global_path_distances
            self.distance_to_velocity_interpolator = distance_to_velocity_interpolator

    def current_velocity_callback(self, msg):
        # save current velocity
        self.current_speed = msg.twist.linear.x

    def current_pose_callback(self, msg):
        # save current pose
        current_position = Point([msg.pose.position.x, msg.pose.position.y])
        self.current_position = current_position

    def detected_objects_callback(self, msg):
        with self.lock:
            global_path_linestring = self.global_path_linestring
            global_path_distances = self.global_path_distances
            distance_to_velocity_interpolator = self.distance_to_velocity_interpolator
            current_position = self.current_position
            current_speed = self.current_speed

        if global_path_linestring is None or global_path_distances is None or distance_to_velocity_interpolator is None or current_position is None or current_speed is None:
            self.publish_local_path_wp([], msg.header.stamp, self.output_frame)
            return

        d_ego_from_path_start = global_path_linestring.project(current_position)
        local_path = self.extract_local_path(global_path_linestring, global_path_distances, d_ego_from_path_start, self.local_path_length)
        
        if local_path is None:
            self.publish_local_path_wp([], msg.header.stamp, self.output_frame)
            return
        
        try:
            transform = self.tf_buffer.lookup_transform("base_link", msg.header.frame_id, msg.header.stamp, rospy.Duration(self.transform_timeout))
        except (TransformException, rospy.ROSTimeMovedBackwardsException) as e:
            rospy.logwarn("%s - %s", rospy.get_name(), e)
            return
        
        target_velocity = distance_to_velocity_interpolator(d_ego_from_path_start)
        local_path_waypoints = self.convert_local_path_to_waypoints(local_path, target_velocity)
        
        local_path_buffer = local_path.buffer(self.stopping_lateral_distance, cap_style="flat")
        prepare(local_path_buffer)
        
        object_distances = []
        object_velocities = []
        object_braking_distances = []
        local_path_blocked = False

        for obj in msg.objects:
            convex_hull = Polygon([Point(p.x, p.y) for p in obj.convex_hull.polygon.points])
        
            if intersects(local_path_buffer, convex_hull):
                min_distance = float('inf')
                for coords in convex_hull.exterior.coords:
                    d = local_path.project(Point(coords))
                    min_distance = min(min_distance, d)
                
                if min_distance <= self.local_path_length:
                    if transform is not None:
                        vector3_stamped = Vector3Stamped(vector=obj.velocity.linear)
                        velocity = do_transform_vector3(vector3_stamped, transform).vector
                    else:
                        velocity = Vector3()
                    
                    object_velocity = velocity.x    
                    
                    object_velocities.append(object_velocity)
                    object_distances.append(min_distance)
                    object_braking_distances.append(self.braking_safety_distance_obstacle+self.current_pose_to_car_front)
                    local_path_blocked = True

        if global_path_linestring.coords[-1] == local_path.coords[-1]:
            object_distances.append(local_path.length)
            object_velocities.append(0)
            object_braking_distances.append(self.braking_safety_distance_goal+self.current_pose_to_car_front)
        
        if len(object_distances) > 0:
            target_distances = np.maximum(np.array(object_distances) - np.array(object_braking_distances) - np.abs(object_velocities)*self.braking_reaction_time, 0)
            target_velocities = np.sqrt(np.power(np.array(object_velocities),2) + 2 * self.default_deceleration * target_distances)
            
            min_target_velocity_index = np.argmin(target_velocities)
            target_velocity = min(target_velocities[min_target_velocity_index], target_velocity)
            
            closest_object_distance = object_distances[min_target_velocity_index] - self.current_pose_to_car_front
            closest_object_velocity = object_velocities[min_target_velocity_index]
            
            stopping_point_distance = target_distances[min_target_velocity_index] + self.current_pose_to_car_front
        else:
            closest_object_distance = 0.0
            closest_object_velocity = 0.0
            stopping_point_distance = 0.0   
            local_path_blocked = False 
        
        local_path_waypoints = self.convert_local_path_to_waypoints(local_path, target_velocity)
        self.publish_local_path_wp(local_path_waypoints, msg.header.stamp, self.output_frame, closest_object_distance, closest_object_velocity, local_path_blocked, stopping_point_distance)

    def extract_local_path(self, global_path_linestring, global_path_distances, d_ego_from_path_start, local_path_length):
        # current position is projected at the end of the global path - goal reached
        if math.isclose(d_ego_from_path_start, global_path_linestring.length):
            return None

        d_to_local_path_end = d_ego_from_path_start + local_path_length

        # find index where distances are higher than ego_d_on_global_path
        index_start = np.argmax(global_path_distances >= d_ego_from_path_start)
        index_end = np.argmax(global_path_distances >= d_to_local_path_end)

        # if end point of local_path is past the end of the global path (returns 0) then take index of last point
        if index_end == 0:
            index_end = len(global_path_linestring.coords) - 1

        # create local path from global path add interpolated points at start and end, use sliced point coordinates in between
        start_point = global_path_linestring.interpolate(d_ego_from_path_start)
        end_point = global_path_linestring.interpolate(d_to_local_path_end)
        local_path = LineString([start_point] + list(global_path_linestring.coords[index_start:index_end]) + [end_point])

        return local_path


    def convert_local_path_to_waypoints(self, local_path, target_velocity):
        # convert local path to waypoints
        local_path_waypoints = []
        for point in local_path.coords:
            waypoint = Waypoint()
            waypoint.pose.pose.position.x = point[0]
            waypoint.pose.pose.position.y = point[1]
            waypoint.pose.pose.position.z = point[2]
            waypoint.twist.twist.linear.x = target_velocity
            local_path_waypoints.append(waypoint)
        return local_path_waypoints


    def publish_local_path_wp(self, local_path_waypoints, stamp, output_frame, closest_object_distance=0.0, closest_object_velocity=0.0, local_path_blocked=False, stopping_point_distance=0.0):
        # create lane message
        lane = Lane()
        lane.header.frame_id = output_frame
        lane.header.stamp = stamp
        lane.waypoints = local_path_waypoints
        lane.closest_object_distance = closest_object_distance
        lane.closest_object_velocity = closest_object_velocity
        lane.is_blocked = local_path_blocked
        lane.cost = stopping_point_distance
        self.local_path_pub.publish(lane)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('simple_local_planner')
    node = SimpleLocalPlanner()
    node.run()