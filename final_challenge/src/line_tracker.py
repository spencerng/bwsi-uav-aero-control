#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import threading
import numpy as np
from datetime import datetime
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import Image
from aero_control.msg import Line
from std_msgs.msg import Float32
from copy import deepcopy

K_P_X = 0.2 # TODO: decide upon initial K_P_X
K_P_Y = 0.095 # TODO: decide upon initial K_P_Y
K_P_Z = 0.02 # TODO: decide upon initial K_P_Z
K_D_Y = 0.25
K_I_Y = 0.0
K_P_ANG_Z = 1.5
K_D_ANG_Z = 0.0
K_I_ANG_Z = 0.0
CENTER = (64, 64)
DIST = 50
TIMEOUT_PERIOD = 1.0

#Responsible of sending velocity commands for line tracking

class LineTracker:

	def __init__(self, rate=10):
		""" Initializes publishers and subscribers, sets initial values for vars
		:param rate: the rate at which the setpoint_velocity is published
		"""
		assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
		self.rate_hz = rate        
		self.rate = rospy.Rate(rate)
		
		
		self.pub_pid_vel = rospy.Publisher("/line_detection/cmd_vel", Vector3, queue_size=1)
		self.sub_line_param = rospy.Subscriber("/line_detection/line", Line, self.line_param_cb)
		
		# Variables dealing with publishing setpoint
		
		self.prev_y_err = 0.0
		self.sum_y_err = 0.0
		self.prev_ang_err = 0.0
		self.sum_ang_err = 0.0
		
		self.velocity_setpoint = TwistStamped()
		self.t_line_last_seen = None
		while True:
			if self.t_line_last_seen is None:
				self.pub_pid_vel.publish(Vector3(0,0,0))
			else:
				self.td = datetime.now() - self.t_line_last_seen
				if self.td.total_seconds()>TIMEOUT_PERIOD and self.td.total_seconds()<2:
					self.pub_pid_vel.publish(Vector3(0,0,0))
					


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
	
	def line_param_cb(self, line_params):
		vx,vy = self.conv_vect(line_params.vx, line_params.vy)
		if vx ==0:
			vx = 0.001
		xc,yc =  self.find_closest_point(line_params.x, line_params.y, vx, vy)
		
		self.ang_err = np.arctan(vy/vx)

		self.pos = self.d_target_position(xc, yc, vx, vy)

		cmd_x, cmd_y, cmd_yaw = self.pid_control(self.pos, self.ang_err)

		self.prev_ang_err = self.ang_err
		self.prev_y_err = self.pos[1]
		self.sum_ang_err+=self.ang_err
		self.sum_y_err+=self.pos[1]
		self.pub_pid_vel.publish(Vector3(cmd_x,cmd_y,cmd_yaw))
		self.t_line_last_seen = datetime.now()
		

if __name__ == "__main__":
	rospy.init_node("line_tracker")
	d = LineTracker()
	rospy.spin()
