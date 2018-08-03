#!/usr/bin/env python
import rospy
import threading
import numpy as np
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Quaternion, Point, Vector3
from std_msgs.msg import Float32
import mavros
from mavros_msgs.msg import State
from copy import deepcopy



NO_ROBOT =True # set to True to test on laptop
MAX_ANG_SPEED = np.pi/2  #[rad/s]
MAX_LIN_SPEED_X = 0.8 # [m/s]
MAX_LIN_SPEED_Y = 0.45 # [m/s]
MAX_LIN_SPEED_Z = .8 # [m/s]
Z_LIN_SPEED = 1.0

class FinalChallengeController:
	def __init__(self, rate=10):
		assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
		self.rate_hz = rate
		self.rate = rospy.Rate(rate)
		self.velocity_setpoint = TwistStamped()
		self.sub_pid_vel = rospy.Subscriber("/line_detection/cmd_vel", Vector3, self.line_vel_cb)
		self.sub_ar_obstacle_vel = rospy.Subscriber("/ar_obstacle/cmd_vel", Float32, self.ar_obstacle_cb)
		self.offboard_point_streaming = False
		self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
		#self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_sub_cb)
		self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)

	def line_vel_cb(self, vel_params):
		self.velocity_setpoint.twist.linear.x = vel_params.x
		self.velocity_setpoint.twist.linear.y = vel_params.y
		self.velocity_setpoint.twist.angular.z = vel_params.z
	def ar_obstacle_cb(self, msg):
		#self.velocity_setpoint.twist.linear.z =  0
		self.velocity_setpoint.twist.linear.z =  -msg.data

	def start_streaming_offboard_points(self):
		""" Starts thread that will publish yawrate at `rate` in Hz
		"""
		def run_streaming():
			self.offboard_point_streaming = True
			rospy.loginfo("Streaming")
			while ((not rospy.is_shutdown()) and self.offboard_point_streaming) or NO_ROBOT:
				#rospy.loginfo("Publishing velocity commands")
				if (self.velocity_setpoint is not None):
					# limit speed for safety
					
					velocity_setpoint_limited = deepcopy(self.velocity_setpoint)
					if np.absolute(velocity_setpoint_limited.twist.linear.x) > MAX_LIN_SPEED_X:
						velocity_setpoint_limited.twist.linear.x = MAX_LIN_SPEED_X * np.sign(velocity_setpoint_limited.twist.linear.x)
					if np.absolute(velocity_setpoint_limited.twist.linear.y) > MAX_LIN_SPEED_Y:
						velocity_setpoint_limited.twist.linear.y = MAX_LIN_SPEED_Y * np.sign(velocity_setpoint_limited.twist.linear.y)


					if np.absolute(velocity_setpoint_limited.twist.linear.z) > MAX_LIN_SPEED_Z:
						velocity_setpoint_limited.twist.linear.z = MAX_LIN_SPEED_Z * np.sign(velocity_setpoint_limited.twist.linear.z)
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
	rospy.init_node("velocity_handler")
	ctrl = FinalChallengeController()
	rospy.loginfo("Starting final challenge controller")
	rospy.spin()
	ctrl.stop_streaming_offboard_points()
