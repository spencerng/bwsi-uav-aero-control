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
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker


NO_ROBOT =True # set to True to test on laptop
MAX_ANG_SPEED = np.pi/2  #[rad/s]
MAX_LIN_SPEED = .5 # [m/s]
DISTANCES = {9:(0.0,0.0,-1.0),12:(0.0,0.0,1.0),24:(0.0,0.0,1.0), 16:(0.0,0.0,1.0)} 
AR_FWD_THRESH = 1.0
AR_FWD_TOL = 0.25
Z_LIN_SPEED = 1.0

class FinalChallengeController:
	def __init__(self, rate=10):
		assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
		self.rate_hz = rate
		self.rate = rospy.Rate(rate)
		self.velocity_setpoint = TwistStamped()
		self.pub_pid_vel = rospy.Subscriber("/line_detection/cmd_vel", Vector3, self.line_vel_cb)
		self.yaw = 0
		self.state = 1
		self.offboard_point_streaming = False
		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb, queue_size = 1)
		self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
		#self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_sub_cb)
		self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
		self.current_marker = None

	
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
		if self.current_marker is not None and self.current_marker.id in DISTANCES.keys():
			ar_tag_pos = self.current_marker.pose.pose.position
			i = self.current_marker.id
				
			err = Vector3(ar_tag_pos.z-DISTANCES[i][0],ar_tag_pos.x-DISTANCES[i][1],ar_tag_pos.y-DISTANCES[i][2])
			rospy.loginfo(err)
			self.pub_error_ar_tag.publish(err)
			

	def line_vel_cb(self, vel_params):
		self.velocity_setpoint.twist.linear.x = vel_params.x
		self.velocity_setpoint.twist.linear.y = vel_params.y
		self.velocity_setpoint.twist.angular.z = vel_params.z

	def set_vel_cmd(self):
		if self.state == 1:
			rospy.loginfo("Following line; looking for AR tag")
			if self.current_marker is not None:
				#self.state +=1
				pass
			#Line follow until correct AR tag is detected within 5 meters
		if self.state == 2:
			while self.current_marker is not None:
				x_err = abs(abs(self.current_marker.pose.pose.position.z)-AR_FWD_THRESH)
				if x_err < AR_FWD_TOL:
					self.state +=1
			# P control the body up X, until within target range of AR tag
			# Minimal, biased input against line detection
		if self.state == 3:
			self.velocity_setpoint.twist.linear.z = Z_LIN_SPEED
			bd_z_err = DISTANCES[self.current_marker.id][2]-self.current_marker.pose.pose.position.y
			duration = (bd_z_err)/Z_LIN_SPEED
			timedelta = datetime.timedelta(seconds = duration)
			
			start_time = datetime.datetime.now()
			while not rospy.is_shutdown() and datetime.now() - start_time < timedelta:
			    rospy.loginfo("flying_up")
		    	self.rate.sleep()
			self.state+=1
			#Complete open loop control, fly up/down using dead reckoning
		if self.state == 4:
			pass
			#OLC with fly forward
		if self.state == 5:
			pass
			# OLC with fly opposite of state 3
			# Should rever bak to state 1

	def start_streaming_offboard_points(self):
		""" Starts thread that will publish yawrate at `rate` in Hz
		"""
		def run_streaming():
			self.offboard_point_streaming = True
			rospy.loginfo("Streaming")
			self.set_vel_cmd()
			while ((not rospy.is_shutdown()) and self.offboard_point_streaming) or NO_ROBOT:
				#rospy.loginfo("Publishing velocity commands")
				if (self.velocity_setpoint is not None):
					# limit speed for safety
					
					velocity_setpoint_limited = deepcopy(self.velocity_setpoint)
					speed = np.linalg.norm([velocity_setpoint_limited.twist.linear.x,
											velocity_setpoint_limited.twist.linear.y,
											velocity_setpoint_limited.twist.linear.z])
					if speed > MAX_LIN_SPEED:
						velocity_setpoint_limited.twist.linear.x *= MAX_LIN_SPEED / speed
						velocity_setpoint_limited.twist.linear.y *= MAX_LIN_SPEED / speed
						velocity_setpoint_limited.twist.linear.z *= MAX_LIN_SPEED / speed
					yaw_cmd = velocity_setpoint_limited.twist.angular.z 
					if np.absolute(yaw_cmd) > MAX_ANG_SPEED:
						yaw_cmd = np.sign(yaw_cmd) * MAX_ANG_SPEED
					velocity_setpoint_limited.twist.angular.z  = yaw_cmd
					# Publish limited setpoint
					self.pub_local_velocity_setpoint.publish(velocity_setpoint_limited)
				self.rate.sleep()
				#self.velocity_setpoint = TwistStamped()
				#self.pub_local_velocity_setpoint.publish(self.velocity_setpoint)
				#set default, no line detected behavior here
				

		self.offboard_point_streaming_thread = threading.Thread(target=run_streaming)
		self.offboard_point_streaming_thread.start()

	def stop_streaming_offboard_points(self):
		""" Safely terminates offboard publisher
		"""
		self.offboard_point_streaming = False
		try:
			self.offboard_point_streaming_thread.join()
		except AttributeError:
			pass
	def state_cb(self, state):
		""" Starts setpoint streamer when mode is "POSCTL" and disables it when mode is "MANUAL"
		:param state: Given by subscribed topic `/mavros/state`
		"""
		self.current_state = state
		mode = getattr(state, "mode", None)
		if (mode == "POSCTL" or NO_ROBOT) and not self.offboard_point_streaming:
			rospy.loginfo("Setpoint stream ENABLED")
			self.start_streaming_offboard_points()
		elif mode == "MANUAL" and self.offboard_point_streaming:
			rospy.loginfo("Setpoint stream DISABLED")
			self.stop_streaming_offboard_points()


if __name__ == "__main__":
	rospy.init_node("final_line_ar_obstacle")
	ctrl = FinalChallengeController()
	rospy.loginfo("Starting final challenge controller")
	rospy.spin()
	ctrl.stop_streaming_offboard_points()
