#!/usr/bin/env python
import rospy
import threading
import numpy as np
import datetime
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import Image
from aero_control.msg import Line
from std_msgs.msg import Float32
import cv2
import mavros
from mavros_msgs.msg import State
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

DISTANCES = {9:(0.0,0.0,-1.0),12:(0.0,0.0,1.0),24:(0.0,0.0,1.0), 16:(0.0,0.0,1.0)} 
AR_FWD_THRESH = 1.0
AR_FWD_TOL = 0.25
K_P_Z
FINAL_CHALLENGE  = True #set true to test on dictionary, false otherwise

class ARObstacleHandler:
	def __init__(self, rate = 10):
		assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
		self.rate_hz = rate        
		self.rate = rospy.Rate(rate)
		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb, queue_size = 1)
		self.mavros_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.mavros_pose_cb, queue_size = 1)
		self.pub_ar_obstacle_vel = rospy.Publisher("/ar_obstacle/cmd_vel", Float32, queue_size=1)
		self.current_pos = None
		pub_ar_obstacle_vel.publish(0.0)


	def mavros_pose_cb(self,msg):
		self.current_pos = msg.pose.position


	def ar_pose_cb(self, msg):
		markers =  msg.markers
		if len(markers) == 0:
			self.current_marker = None
		else:
			for marker in markers:
				if marker.id in DISTANCES:
					rospy.loginfo("Valid AR marker received")
			min_dist = markers[0].pose.pose.position.z
			self.current_marker = markers[0]
			for marker in markers:
				current_dist = marker.pose.pose.position.z
				if current_dist < min_dist and marker.id in DISTANCES:
					self.current_marker = marker
					min_dist = current_dist
		if self.current_marker is not None and (FINAL_CHALLENGE or self.current_marker.id in DISTANCES.keys()):
			ar_tag_pos = self.current_marker.pose.pose.position
			i = self.current_marker.id
				
			err = ar_tag_pos.z-DISTANCES[i][2]-ar_tag_pos.x-DISTANCES[i][1]
			rospy.loginfo(err)
			self.pub_error_ar_tag.publish(err)
	


if __name__ == "__main__":
	rospy.init_node("ar_obstacle_avoid_tracker")
	ctrl = ARObstacleHandler()

	rospy.spin()
