#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
import math

from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

_Z_THRESH = 0.1


# may  need to run rosrun ar_track_alvar createMarker or roslaunch the script

class ARDistChecker:

	def __init__(self):
		rospy.loginfo("ARDistChecker Started!")

		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb)

		self.seen = {}
		self.current_marker = None

	def ar_pose_cb(self, msg):
		
		#Filters AR tags to find the one with shortest distance (in front camera Z)
		if len(msg.markers) > 0:
			min_dist = msg.markers[0].pose.pose.position.z
			self.current_marker = msg.markers[0]
			for marker in msg.markers:
				current_dist = marker.pose.pose.position.z
				if current_dist < min_dist:
					self.current_marker = marker
					min_dist = current_dist

			self.check_dist()

	def within_range(self, dist, goal, tol):
		return abs(dist-goal) < tol:

	def check_dist(self):

		marker = self.current_marker
		z_dist = marker.pose.pose.position.z

		z_des = self.get_dist(marker.id)
		tol = 0.1


		if marker.id in self.seen.keys() and self.within_range(z_dist, z_des, tol):
			rospy.loginfo("Already seen: marker " + str(marker.id))

		elif self.within_range(z_dist, z_des, tol):
			rospy.loginfo("Got it: marker " + str(marker.id) + " captured")
			self.seen[marker.id] = marker

		elif marker.id not in self.seen.keys():
			rospy.loginfo("Not there yet: move " + str(min(abs(1.1 - z_dist), abs(0.9 - z_dist))) + " meters to capture " + str(
					marker.id))

	def get_dist(self, id):
		return 1.0  # change later


if __name__ == '__main__':
	rospy.init_node('ar_checker')
	a = ARDistChecker()

	rospy.spin()
