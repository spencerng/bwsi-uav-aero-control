#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import threading
import numpy as np
import datetime
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import Image
from aero_control.msg import Line
import cv2
import mavros
from mavros_msgs.msg import State
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy

NO_ROBOT = False # set to True to test on laptop
MAX_ANG_SPEED = np.pi/2  #[rad/s]
MAX_LIN_SPEED = .5 # [m/s]
K_P_X = 0.05 # TODO: decide upon initial K_P_X
K_P_Y = 0.02 # TODO: decide upon initial K_P_Y
K_D_Y = 0.012
K_I_Y = 0.0
K_P_ANG_Z = 1.5
K_D_ANG_Z = 0.0
K_I_ANG_Z = 0.0
CENTER = (64, 64)
DIST = 50

class LineTracker:
	@staticmethod
	def conv_vect(v_x, v_y):
		if v_x<0:
		  v_x = -1*v_x
		  v_y = -1*v_y
		return(v_x, v_y)
	@staticmethod
	def find_closest_point(x, y, v_x, v_y):
		m_reg = v_y/v_x
		b_reg = y - m_reg * x
		if m_reg == 0:
			m_reg = 0.0001
		b_perp = CENTER[1]+CENTER[0]/m_reg
		#This is the point of intersection between the line perpendicular to the received line
		#containing the drone's position and the received line
		xc = ((m_reg*(b_perp-b_reg)) / (m_reg**2 +1))
		yc =-xc/m_reg + b_perp 
		return xc, yc
	@staticmethod
	def find_error(xc, yc):
		return (xc-CENTER[0], yc-CENTER[1])
	@staticmethod
	def d_target_position(xc, yc, v_x, v_y):
		#Gives distance from drone current position to target point
		#Target point is DIST away from the closest point, in the +x direction
		theta=np.arctan(v_y/v_x)
		x_f = xc + DIST*np.cos(theta)
		y_f = yc + DIST*np.sin(theta)
		return(x_f - CENTER[0], y_f - CENTER[1])

	def pid_control(self, pos, ang_err):

		vel_cmd_x =  K_P_X * pos[0]
		dt = 1.0/self.rate_hz
		vel_cmd_y =  -(K_P_Y * pos[1]+ K_D_Y * (pos[1]-self.prev_y_err)/dt  + K_I_Y * self.sum_y_err) #Set negative due to BU frame of reference compared to downward camera

		yaw_cmd = - (K_P_ANG_Z * ang_err + K_D_ANG_Z * (ang_err-self.prev_ang_err)/dt + K_I_ANG_Z * self.sum_ang_err)
		
		return (vel_cmd_x,vel_cmd_y, yaw_cmd)
	def __init__(self, rate=10):
		""" Initializes publishers and subscribers, sets initial values for vars
		:param rate: the rate at which the setpoint_velocity is published
		"""
		assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
		self.rate_hz = rate        
		self.rate = rospy.Rate(rate)
		self.prev_ang_err = 0.0
		mavros.set_namespace()
		self.bridge = CvBridge()

		self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
		self.sub_line_param = rospy.Subscriber("/line_detection/line", Line, self.line_param_cb)
		self.pub_error = rospy.Publisher("/line_detection/error", Vector3, queue_size=1)
	

		# Variables dealing with publishing setpoint
		self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
		self.current_state = None
		self.offboard_point_streaming = False
		self.prev_y_err = 0.0
		self.sum_y_err = 0.0
		self.sum_ang_err = 0.0
		# Setpoint field expressed as the desired velocity of the body-down frame
		#  with respect to the world frame parameterized in the body-down frame
		self.velocity_setpoint = None

		while not rospy.is_shutdown() and self.current_state == None:
			pass  # Wait for connection

	def line_param_cb(self, line_params):
		mode = getattr(self.current_state, "mode", None)
		if (mode is not None and mode != "MANUAL") or NO_ROBOT:
			""" Map line paramaterization to a velocity setpoint so the robot will approach and follow the LED strip
			
			Note: Recall the formatting of a Line message when dealing with line_params

			Recomended Steps: 
			
			Read the documentation at https://bwsi-uav.github.io/website/line_following.html

			After calculating your various control signals, place them in self.velocity_setpoint (which
				is a TwistStamped, meaning self.velocity_setpoint.twist.linear.x is x vel for example)

			Be sure to publish your error using self.pub_error.publish(Vector3(x_error,y_error,0))
	
			"""
	
			vx,vy = self.conv_vect(line_params.vx, line_params.vy)
			if vx ==0:
				vx = 0.001
			print("Velocities:",vx,vy)
			xc,yc =  self.find_closest_point(line_params.x, line_params.y, vx, vy)

			print("Point closest to line:",xc,yc)
			
			ang_err = np.arctan(vy/vx)

			pos = self.d_target_position(xc, yc, vx, vy)
			print("Target point deltas/Error:",pos)
			self.pub_error.publish(Vector3(pos[0],pos[1],ang_err))

			self.sum_y_err += pos[1]/10
			self.sum_ang_err += ang_err
	#        print("Actuator Velocities:",self.pid_control(pos, ang_err))
			self.velocity_setpoint = TwistStamped()
			self.velocity_setpoint.twist.linear.x, self.velocity_setpoint.twist.linear.y, self.velocity_setpoint.twist.angular.z = self.pid_control(pos, ang_err)
			#self.velocity_setpoint.twist.linear.z = 0
			self.velocity_setpoint.twist.angular.x = 0
			self.velocity_setpoint.twist.angular.y = 0
			self.prev_ang_err = ang_err
			self.prev_y_err = pos[1]
			
			# TODO-START: Create velocity controller based on above specs
			#raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
			# TODO-END

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

	def start_streaming_offboard_points(self):
		""" Starts thread that will publish yawrate at `rate` in Hz
		"""
		def run_streaming():
			self.offboard_point_streaming = True
			while (not rospy.is_shutdown()) and self.offboard_point_streaming:
				# Publish commands
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


if __name__ == "__main__":
	rospy.init_node("line_tracker")
	d = LineTracker()
	rospy.spin()
	d.stop_streaming_offboard_points()
