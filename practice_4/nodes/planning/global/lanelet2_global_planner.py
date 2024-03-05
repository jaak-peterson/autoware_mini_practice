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

class Lanelet2GlobalPlanner:
    def __init__(self):
        self.current_location = None
        self.goal_point = None
        self.graph = None

        # Parameters
        lanelet2_map_name = rospy.get_param("~lanelet2_map_name")
        coordinate_transformer = rospy.get_param("/localization/coordinate_transformer")
        use_custom_origin = rospy.get_param("/localization/use_custom_origin")
        utm_origin_lat = rospy.get_param("/localization/utm_origin_lat")
        utm_origin_lon = rospy.get_param("/localization/utm_origin_lon")

        self.speed_limit = rospy.get_param("~speed_limit")
        self.output_frame = rospy.get_param("/waypoint_loader/output_frame")
        self.distance_to_goal_limit = rospy.get_param("/lanelet2_global_planner/distance_to_goal_limit")

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
        # loginfo message about receiving the goal point
        rospy.loginfo("%s - goal position (%f, %f, %f) orientation (%f, %f, %f, %f) in %s frame", rospy.get_name(),
                    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z,
                    msg.pose.orientation.w, msg.header.frame_id)
        self.goal_point = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        # get start and end lanelets
        start_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.current_location, 1)[0][1]
        goal_lanelet = findNearest(self.lanelet2_map.laneletLayer, self.goal_point, 1)[0][1]
        # find routing graph
        route = self.graph.getRoute(start_lanelet, goal_lanelet, 0, True)

        if not route:
            rospy.logwarn("No route found")
            self.goal_point = None
            return None
        # find shortest path
        path = route.shortestPath()
        # this returns LaneletSequence to a point where lane change would be necessary to continue
        path_no_lane_change = path.getRemainingLane(start_lanelet)
        self.lanelet_to_waypoints(path_no_lane_change)
        
    def current_pose_callback(self, msg):
        self.current_location = BasicPoint2d(msg.pose.position.x, msg.pose.position.y)

        if self.goal_point is not None:
            distance_to_goal = np.sqrt((self.goal_point.x - self.current_location.x)**2 + (self.goal_point.y - self.current_location.y)**2)
            if distance_to_goal < self.distance_to_goal_limit:
                rospy.loginfo("Goal reached")
                self.lanelet_to_waypoints([])

    def lanelet_to_waypoints(self, path):
        waypoints = []
        isLast = False
        for index, lanelet in enumerate(path):
            if index == len(path) - 1:
                isLast = True
            # code to check if lanelet has attribute speed_ref
            if 'speed_ref' in lanelet.attributes:
                speed = float(lanelet.attributes['speed_ref'])
            else:
                speed = self.speed_limit
            
            # Check if speed is not greater than speed limit
            speed = min(speed, self.speed_limit) / 3.6
            
            for i in range(len(lanelet.centerline)):
                point = lanelet.centerline[i]
                # create Waypoint and get the coordinats from lanelet.centerline points
                if i != len(lanelet.centerline) - 1 or isLast:
                    waypoint = Waypoint()
                    waypoint.pose.pose.position.x = point.x
                    waypoint.pose.pose.position.y = point.y
                    waypoint.pose.pose.position.z = point.z
                    waypoint.twist.twist.linear.x = speed
                    waypoints.append(waypoint)

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