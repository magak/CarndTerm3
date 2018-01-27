#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pos = msg

	closest_waypointIndex = self.get_closest_waypoint()
	if (self.baseWaypoints is None) or (closest_waypointIndex is None) or len(self.baseWaypoints.waypoints)==0:
		return	

	lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
	
	wayp = self.baseWaypoints.waypoints[closest_waypointIndex:min(len(self.baseWaypoints.waypoints), 
		closest_waypointIndex+LOOKAHEAD_WPS-1)]
	#wayp.twist.twist.angular.x = self.current_pos.pose.position.x
	#wayp.twist.twist.angular.y = self.current_pos.pose.position.y

        lane.waypoints = wayp
	self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
	self.baseWaypoints = waypoints

	#velocity = self.get_waypoint_velocity(self.baseWaypoints.waypoints[450])
	#for i in range(450, len(self.baseWaypoints.waypoints)):
	#	velocity = max(velocity - 0.8, 0.0)
	#	self.set_waypoint_velocity(self.baseWaypoints.waypoints, i, velocity)

	#for i in range(len(self.baseWaypoints.waypoints)):
	#	velocity = 0.0
	#	self.set_waypoint_velocity(self.baseWaypoints.waypoints, i, velocity)
		

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_closest_waypoint(self):
	if (self.current_pos is None) or (self.baseWaypoints is None) or len(self.baseWaypoints.waypoints)==0:
		return None
	
	closest = np.argmin([self.distance(self.current_pos.pose.position, item.pose.pose.position) 
					for item in self.baseWaypoints.waypoints])
	# if the point is behind the current position we will take the next one
	
	if(closest < len(self.baseWaypoints.waypoints)-1):
		pClosest = np.array([self.baseWaypoints.waypoints[closest].pose.pose.position.x
					,self.baseWaypoints.waypoints[closest].pose.pose.position.y])
		pAfterClosest = np.array([self.baseWaypoints.waypoints[closest+1].pose.pose.position.x
					,self.baseWaypoints.waypoints[closest+1].pose.pose.position.y])	
		pCurrent = np.array([self.current_pos.pose.position.x
					,self.current_pos.pose.position.y])
		if(np.dot(pClosest-pCurrent, pAfterClosest-pCurrent)>0):
			return closest
		else:
			return closest+1
	else:
		return None
		

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def dot(self, p1, p2):
        return p1.x * p2.x + p1.y * p2.y 

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
