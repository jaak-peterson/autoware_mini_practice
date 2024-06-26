#!/usr/bin/env python3

import rospy
import numpy as np
import threading

from autoware_msgs.msg import Lane, VehicleCmd
from geometry_msgs.msg import PoseStamped
from shapely.geometry import LineString, Point
from shapely import prepare, distance
from tf.transformations import euler_from_quaternion
from scipy.interpolate import interp1d

class PurePursuitFollower:
    def __init__(self):
        # Parameters
        self.lookahead_distance = rospy.get_param("~lookahead_distance")
        self.wheel_base = rospy.get_param("/wheel_base")
        
        # Global variables
        self.path_linestring = None
        self.distance_to_velocity_interpolator = None

        # Locks
        self.lock = threading.Lock()

        # Publishers
        self.vehicle_cmd_pub = rospy.Publisher('/control/vehicle_cmd', VehicleCmd, queue_size=10)

        # Subscribers
        rospy.Subscriber('path', Lane, self.path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def path_callback(self, msg):
        # convert waypoints to shapely linestring
        coordinates = [(w.pose.pose.position.x, w.pose.pose.position.y) for w in msg.waypoints]
        path_linestring = LineString(coordinates)
        waypoints_xy = np.array(coordinates)

        # prepare path - creates spatial tree, making the spatial queries more efficient
        prepare(path_linestring)
        
        # create distance to velocity interpolator
        distances = np.cumsum(np.sqrt(np.sum(np.diff(waypoints_xy, axis=0)**2, axis=1)))
        distances = np.insert(distances, 0, 0)
        velocities = np.array([w.twist.twist.linear.x for w in msg.waypoints])

        distance_to_velocity_interpolator_local = interp1d(distances, velocities, kind='linear', bounds_error=True, fill_value=0.0)
        with self.lock:
            self.path_linestring = path_linestring
            self.distance_to_velocity_interpolator = distance_to_velocity_interpolator_local

    def current_pose_callback(self, msg):
        with self.lock:
            path_linestring_local = self.path_linestring
            distance_to_velocity_interpolator_local = self.distance_to_velocity_interpolator

        if path_linestring_local is not None and distance_to_velocity_interpolator_local is not None:
            current_pose = Point([msg.pose.position.x, msg.pose.position.y])
            d_ego_from_path_start = path_linestring_local.project(current_pose)

            lookahead_point = path_linestring_local.interpolate(self.lookahead_distance + d_ego_from_path_start)
            ld = distance(current_pose, lookahead_point)
            _, _, heading = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

            # lookahead point heading calculation
            lookahead_heading = np.arctan2(lookahead_point.y - current_pose.y, lookahead_point.x - current_pose.x)
            heading_diff = lookahead_heading - heading
            steering_angle = np.arctan((2*self.wheel_base * np.sin(heading_diff))/ld)
            
            velocity = distance_to_velocity_interpolator_local(d_ego_from_path_start)

            vehicle_cmd = VehicleCmd()
            vehicle_cmd.header.stamp = msg.header.stamp
            vehicle_cmd.header.frame_id = "base_link"
            vehicle_cmd.ctrl_cmd.steering_angle = steering_angle
            vehicle_cmd.ctrl_cmd.linear_velocity = velocity
            self.vehicle_cmd_pub.publish(vehicle_cmd)
        return
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_follower')
    node = PurePursuitFollower()
    node.run()