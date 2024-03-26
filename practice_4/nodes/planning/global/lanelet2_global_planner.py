#!/usr/bin/env python3

import rospy
import numpy as np

# All these imports from lanelet2 library should be sufficient
import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector
from lanelet2.core import BasicPoint2d
from lanelet2.geometry import findNearest
from geometry_msgs.msg import PoseStamped
from autoware_msgs.msg import Lane, Waypoint
from shapely import Point, LineString

class Lanelet2GlobalPlanner:
    def __init__(self):
        # Parameters
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.speed_limit = rospy.get_param("~speed_limit")
        self.output_frame = rospy.get_param("/waypoint_loader/output_frame")
        self.distance_to_goal_limit = rospy.get_param("/lanelet2_global_planner/distance_to_goal_limit")
        
        # Global variables
        self.current_location = None
        self.goal_point = None

        # Load the map using Lanelet2
        if coordinate_transformer == "utm":
            projector = UtmProjector(Origin(utm_origin_lat, utm_origin_lon), use_custom_origin, False)
        else:
            raise RuntimeError('Only "utm" is supported for lanelet2 map loading')
        self.lanelet2_map = load(lanelet2_map_name, projector)

        # traffic rules
        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                lanelet2.traffic_rules.Participants.VehicleTaxi)
        # routing graph
        self.graph = lanelet2.routing.RoutingGraph(self.lanelet2_map, traffic_rules)

        # Publishers
        self.waypoints_pub = rospy.Publisher('global_path', Lane, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.global_path_callback, queue_size=1)
        rospy.Subscriber('/localization/current_pose', PoseStamped, self.current_pose_callback, queue_size=1)

    def global_path_callback(self, msg):
        if self.current_location is None:
            rospy.logwarn("%s - Current location not received", rospy.get_name())
            return

        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w, msg.header.frame_id)
        
        self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)

        if not route:
            rospy.logwarn("%s - No route found", rospy.get_name())
            self.goal_point = None
            return None
        path = route.shortestPath()
        path_no_lane_change = path.getRemainingLane(start_lanelet)
        self.publish_waypoints(self.lanelet_to_waypoints(path_no_lane_change))
        
    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)
        
        if self.goal_point is None:
            return

        if self.goal_point is not None:
            distance_to_goal = np.sqrt((self.goal_point.x - self.current_location.x)**2 + (self.goal_point.y - self.current_location.y)**2)
            if distance_to_goal < self.distance_to_goal_limit:
                rospy.loginfo("%s - Goal reached", rospy.get_name())
                self.goal_point = None
                self.publish_waypoints([])

    def lanelet_to_waypoints(self, path):
        waypoints = []

        isLast = False
        for index, lanelet in enumerate(path):
            if index == len(path) - 1:
                isLast = True
            if 'speed_ref' in lanelet.attributes:
                speed = float(lanelet.attributes['speed_ref'])
            else:
                speed = self.speed_limit
            
            speed = min(speed, self.speed_limit) / 3.6
            
            for i in range(len(lanelet.centerline)):
                point = lanelet.centerline[i]
                if i != len(lanelet.centerline) - 1 or isLast:
                    waypoint = Waypoint()
                    waypoint.pose.pose.position.x = point.x
                    waypoint.pose.pose.position.y = point.y
                    waypoint.pose.pose.position.z = point.z
                    waypoint.twist.twist.linear.x = speed
                    waypoints.append(waypoint)
        
        goal_point = Point([self.goal_point.x, self.goal_point.y])
        waypoints_array = np.array([(wp.pose.pose.position.x, wp.pose.pose.position.y, wp.pose.pose.position.z) for wp in waypoints])
        waypoints_linestring = LineString(waypoints_array[:, :2])

        distance_to_goal = waypoints_linestring.project(goal_point)
        goal_point_in_waypoints = waypoints_linestring.interpolate(distance_to_goal)

        min_distance = float('inf')
        min_distance_index = 0
        for i, wp in enumerate(waypoints_array):
            distance = np.sqrt((goal_point_in_waypoints.x - wp[0])**2 + (goal_point_in_waypoints.y - wp[1])**2)
            if distance < min_distance:
                min_distance = distance
                min_distance_index = i

        closest_waypoint = waypoints_array[min_distance_index]

        goal_waypoint = Waypoint()
        goal_waypoint.pose.pose.position.x = goal_point_in_waypoints.x
        goal_waypoint.pose.pose.position.y = goal_point_in_waypoints.y
        goal_waypoint.pose.pose.position.z = closest_waypoint[2]

        self.goal_point = BasicPoint2d(goal_waypoint.pose.pose.position.x, goal_waypoint.pose.pose.position.y)
        
        filtered_waypoints = []
        for wp in waypoints:
            wp_point = Point([wp.pose.pose.position.x, wp.pose.pose.position.y])
            distance = waypoints_linestring.project(wp_point)
            if distance < distance_to_goal:
                filtered_waypoints.append(wp)    

        filtered_waypoints.append(goal_waypoint)
        
        return filtered_waypoints

    def publish_waypoints(self, waypoints):
        lane = Lane()        
        lane.header.frame_id = self.output_frame
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        self.waypoints_pub.publish(lane)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lanelet2_global_planner')
    node = Lanelet2GlobalPlanner()
    node.run()