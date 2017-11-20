#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
# ADD OTHER GLOBALS HERE
LARGE = 1.e10

def euc_dist(x1,y1,x2,y2):
	x = (x1-x2)
	y = (y1-y2)
	return math.sqrt(x*x+y*y)


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) ## rosnode info /waypoint_updater displays this as having unknown type.  Why should this be?
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_wp = []
        self.final_wp = []
        self.next_wp_idx = 0
        
        rospy.spin()

    ## ADD PUBLISHER for final_wp
    def publish(self):
    	lane = Lane()
    	#lane.header.frame_id = '' # this will need to be filled in
    	lane.header.stamp = rospy.Time(0)
    	lane.waypoints = self.final_wp
    	self.final_waypoints_pub.publish(lane)

    ## helpers
    def get_next_waypoint_idx(self):
    	pos_x = self.pose.position.x
        pos_y = self.pose.position.y

        # iterate over waypoints to find the next one (i.e. closest in front)
        min_dist = LARGE
        next_idx = 0
        for idx in range(len(self.base_wp)):
            wp_x = self.base_wp[idx].pose.pose.position.x
            wp_y = self.base_wp[idx].pose.pose.position.y
            if euc_dist(wp_x, wp_y, pos_x, pos_y) < min_dist:
                min_dist = euc_dist(wp_x, wp_y, pos_x, pos_y)
                next_idx = idx

        return next_idx


    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose

        # get next waypoint index
        self.next_wp_idx = self.get_next_waypoint()

        # look forward to minimum of (LOOKAHEAD_WPS, size(base_wp))
        end_idx = min(self.next_wp_idx + LOOKAHEAD_WPS - 1, len(base_wp))
        wp = self.base_wp[self.next_wp_idx : end_idx]

        self.final_wp = wp
        self.publish()


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        # set waypoints
        self.base_wp = waypoints 

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

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
