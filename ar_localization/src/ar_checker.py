#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

_Z_THRESH = 0.1


#may  need to run rosrun ar_track_alvar createMarker or roslaunch the script

class ARDistChecker:

    
    def __init__(self):
        rospy.loginfo("ARDistChecker Started!")

        '''
        TODO: Determine how to initialize a subscriber for AR tracking
        '''
        self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)


        self.seen = {}
        self.current_marker = None


    def ar_pose_cb(self,msg):
        '''
        TODO: Filter incoming AR message to determine where drone is relative to tag
        '''
	if len(msg.markers)>0:
		min_dist = msg.markers[0].pose.pose.position.z
		self.current_marker = msg.markers[0]
		for marker in msg.markers:
		    current_dist = marker.pose.pose.position.z
		    if current_dist < min_dist:
		        self.current_marker = marker
		        min_dist=current_dist
		    
		self.check_dist()

    def check_dist(self):

        '''
        TODO: Determine how to share marker data with check_dist
        '''
        marker = self.current_marker

        z_dist = self.get_dist(marker.id)


        '''
        TODO: Fill in conditionals appropriately to filter marker cases
        '''
        if marker.id in self.seen.keys() and 1.1 > z_dist > 0.9:
            rospy.loginfo("Already seen: marker "+ str(marker.id))

        elif 1.1 > z_dist > 0.9:
            rospy.loginfo("Got it: marker " + str(marker.id) + " captured")
            self.seen[marker.id] = marker
            # MINI TODO: How can we track successful detects? (HINT: add something to this elif block)

        else:
            rospy.loginfo("Not there yet: move " + str(z_dist) + " meters to capture " + str(marker.id))
            # MINI TODO: replace None with a method of calculating distance to AR tag

        

    def get_dist(self,id):
        return 1.0 #change later
        


if __name__ == '__main__':
    rospy.init_node('ar_checker')
    a = ARDistChecker()

    rospy.spin()



