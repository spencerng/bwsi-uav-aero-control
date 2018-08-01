#!/usr/bin/env python
from __future__ import division, print_function
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

NO_ROBOT = False # set to True to test on laptop
MAX_ANG_SPEED = np.pi/2  #[rad/s]
MAX_LIN_SPEED = .5 # [m/s]
K_P_X = 0.05 # TODO: decide upon initial K_P_X
K_P_Y = 0.02 # TODO: decide upon initial K_P_Y
K_P_Z = 0.02 # TODO: decide upon initial K_P_Z
K_D_Y = 0.012
K_I_Y = 0.0
K_P_ANG_Z = 1.5
K_D_ANG_Z = 0.0
K_I_ANG_Z = 0.0
BIAS_XY = 0.0 #how much to fly in each direction when avoiding obstacles
BIAS_Z = 1.0
CENTER = (64, 64)
DIST = 50
MAX_DIST_TO_OBSTACLE = 1.25
TOL = 0.1
DISTANCES = {9:(0.0,0.0,-1.0),12:(0.0,0.0,1.0),24:(0.0,0.0,1.0), 16:(0.0,0.0,1.0)} # height distance +z of hole relative to AR tag in 'bu' or -y in 'fc'  TODO - set distances.
FWD_DISTANCES = {9:0.4,12:0.4,24:0.4,16:0.4} 
_SPEED = 0.5 # Fly through/around obstacle and reset position speed - still limited by MAX_LIN_SPEED

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



	def check_dist(self):
		'''
		Shares marker data with check_dist
		'''
		marker = self.current_marker
		return marker.pose.pose.position.z
        
	def within_range(self):
		if self.current_marker is None:
			pass
		else:
			rospy.loginfo("Distance to obstacle: " + str(abs(self.check_dist())))
		return ((self.current_marker is not None) and (abs(self.check_dist()) < MAX_DIST_TO_OBSTACLE))
			
	
	def fly_up_obstacle(self, identity, pos, ang_err):# diagonal fly forward: get into position
	    	# timedelta (datetime.timedelta object) is the amount of time the velocity message will be published for
		# BD: (reference purposes) (all signs below in fly_up_obstacle should be flipped according to the comment below)
		#	x_bd = x_bu		
		# 	y_bd = -y_bu
		#	z_bd = -z_bu	
		#	yaw = -yaw_bu
        x_mark = -self.current_marker.pose.pose.position.z
        y_mark = -self.current_marker.pose.pose.position.x
        z_mark = -self.current_marker.pose.pose.position.y 
		self.isflying_up = True
		rospy.loginfo("FLying up")
		duration = (DISTANCES[identity][2]+z_mark)/_SPEED
		timedelta = datetime.timedelta(seconds = duration)
		# Record the start time
		start_time = datetime.datetime.now()
		# Publish command velocites for timedelta seconds
		self.velocity_setpoint.twist.linear.z = -_SPEED 
		self.velocity_setpoint.twist.angular.x = 0
		self.velocity_setpoint.twist.angular.y = 0
		while not rospy.is_shutdown() and datetime.datetime.now() - start_time < timedelta and (NO_ROBOT or self.current_state.mode == 'OFFBOARD'):
		    pid_x, pid_y, pid_yaw = self.pid_control(pos, ang_err)
		    self.velocity_setpoint.twist.linear.x = pid_x * BIAS_XY
		    self.velocity_setpoint.twist.linear.y = -pid_y * BIAS_XY
		    self.velocity_setpoint.twist.angular.z = -pid_yaw  * BIAS_XY
		    rospy.loginfo("flying_up")
		    self.rate.sleep()
		# at end of maneuver, set setpoint back to zero
		self.isflying_up = False
		#TODO
		#if X distance is very small in BU (or Z in FC) to AR tag, X velocity in BU should be minimal
		#Possible solution: heavily bias Z error in final velocity vector

		'''
		Done = TODO ******************************************************************
		'''
		# timedelta (datetime.timedelta object) is the amount of time the velocity message will be published for
		self.isflying_forward = True
		rospy.loginfo("Flying forward")
		duration = (FWD_DISTANCES[identity] + x_mark)/_SPEED
		timedelta = datetime.timedelta(seconds = duration)
		# Record the start time
		start_time = datetime.datetime.now()
		 # Publish command velocites for timedelta seconds
		self.velocity_setpoint.twist.linear.x = _SPEED
		self.velocity_setpoint.twist.linear.y = 0
		self.velocity_setpoint.twist.linear.z = 0
		self.velocity_setpoint.twist.angular.x = 0
		self.velocity_setpoint.twist.angular.y = 0
		self.velocity_setpoint.twist.angular.z = 0 #should we correct yaw based on position?
		while not rospy.is_shutdown() and datetime.datetime.now() - start_time < timedelta and  (NO_ROBOT or self.current_state.mode == 'OFFBOARD'):
		    # Note: we don't actually have to call the publish command here.
		    # Publish velocity command is automatically done in run_streaming function which is running in parallel
		    # and just publishes whatever is stored in self.velocity_setpoint
		    rospy.loginfo("flying_fwd")
		    self.rate.sleep()
		# at end of maneuver, set setpoint back to zero
		self.isflying_forward = False
		'''
		Done = TODO move fwd_distances in x *************
		'''
		# timedelta (datetime.timedelta object) is the amount of time the velocity message will be published for
		self.isflying_reset = True
		duration = (DISTANCES[identity][2]+z_mark)/_SPEED
		timedelta = datetime.timedelta(seconds = duration)
		# Record the start time
		start_time = datetime.datetime.now()
		# Publish command velocites for timedelta seconds
		self.velocity_setpoint.twist.linear.z = _SPEED
		self.velocity_setpoint.twist.angular.x = 0
		self.velocity_setpoint.twist.angular.y = 0
		while not rospy.is_shutdown() and datetime.datetime.now() - start_time < timedelta and  (NO_ROBOT or self.current_state.mode == 'OFFBOARD'):
		    pid_x, pid_y, pid_yaw = self.pid_control(pos, ang_err)
		    self.velocity_setpoint.twist.linear.x = pid_x * BIAS_XY
		    self.velocity_setpoint.twist.linear.y = -pid_y * BIAS_XY
		    self.velocity_setpoint.twist.angular.z = -pid_yaw * BIAS_XY
		    rospy.loginfo("flying_reset")
		    self.rate.sleep()

		# at end of maneuver, set setpoint back to zero
		self.isflying_reset = False
		'''
		TODO possibly use distance sensor ******************************************************************
		'''
	        self.current_marker = None
		rospy.loginfo("Marker_reset")
		self.velocity_setpoint = TwistStamped()
	#def fly_forward(self, identity): # fly through/around obstacle
		

	#def reset_pos(self, identity, pos, ang_err): #diagonal segway into line_follow flightpath
		


		
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
		rospy.loginfo("starting up ar obstacle avoidence w/ line follow!")
		self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)
		self.sub_line_param = rospy.Subscriber("/line_detection/line", Line, self.line_param_cb)
		self.pub_error = rospy.Publisher("/line_detection/error", Vector3, queue_size=1)
	        self.pub_error_ar_tag = rospy.Publisher("/line_detection/Zerror", Vector3, queue_size=1)
		'''	Initializes a subscriber for AR tracking'''
		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb, queue_size = 1)
		self.current_marker = None

		# Variables dealing with publishing setpoint
		self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
		self.current_state = None
		self.offboard_point_streaming = False
		self.prev_y_err = 0.0
		self.sum_y_err = 0.0
		self.sum_ang_err = 0.0
		self.pos = (0,0,0)
		self.ang_err = 0
		self.isflying_up = False
		self.isflying_forward = False
		self.isflying_reset = False

		# Setpoint field expressed as the desired velocity of the body-down frame
		#  with respect to the world frame parameterized in the body-down frame
		self.velocity_setpoint = TwistStamped()
		#self.start_streaming_offboard_points()
		#self.fly_up_obstacle(16, (0,0,0), 0)

		while not rospy.is_shutdown() and self.current_state == None:
			pass  # Wait for connection
	def vel_ctrl(self):
#		print("Target point deltas/Error:",pos)
		
		self.pub_error.publish(Vector3(self.pos[0],self.pos[1],self.ang_err))
		self.sum_y_err += self.pos[1]/10
		self.sum_ang_err += self.ang_err
	#       print("Actuator Velocities:",self.pid_control(pos, ang_err))
		self.velocity_setpoint = TwistStamped()
		pid_x, pid_y, pid_yaw = self.pid_control(self.pos, self.ang_err)
		if(not self.within_range()and not (self.isflying_up or self.isflying_forward or self.isflying_reset)): #line_tracking
			self.velocity_setpoint.twist.linear.x = pid_x
			self.velocity_setpoint.twist.linear.y = pid_y
			self.velocity_setpoint.twist.angular.z = pid_yaw
			self.velocity_setpoint.twist.linear.z = 0
		elif self.current_marker is not None:
			rospy.loginfo("Attempting open loop control")
			if not (self.isflying_up or self.isflying_forward or self.isflying_reset):
				self.fly_up_obstacle(self.current_marker.id, self.pos, self.ang_err)
			#if not self.isflying_up:
			#	self.fly_forward(self.current_marker.id) #fly through/around obstacle
			#if not (self.isflying_forward or self.isflying_up):
						# RESET position and marker
			#	self.reset_pos(self.current_marker.id, self.pos, self.ang_err)
			self.current_marker = None
		     
			
		self.prev_ang_err = self.ang_err
		self.prev_y_err = self.pos[1]
		if not (self.isflying_forward or self.isflying_up or self.isflying_reset):
			self.ang_err = 0
			self.pos = (0,0,0)
	def ar_pose_cb(self, msg):
		'''
		Filtering incoming AR message to determine where drone is relative to tag
		'''
		self.current_marker = None
		if len(msg.markers) > 0:
			for marker in msg.markers:
				if marker.id in DISTANCES.keys():
					rospy.loginfo("AR marker(s) received")
			min_dist = msg.markers[0].pose.pose.position.z
			self.current_marker = msg.markers[0]
			for marker in msg.markers:
				current_dist = marker.pose.pose.position.z
				if current_dist < min_dist and marker.id in DISTANCES.keys():
					self.current_marker = marker
					min_dist = current_dist
		if self.current_marker is not None and self.current_marker.id in DISTANCES.keys():
			ar_tag_pos = self.current_marker.pose.pose.position
			i = self.current_marker.id
				
			err = Vector3(ar_tag_pos.z-DISTANCES[i][0],ar_tag_pos.x-DISTANCES[i][1],ar_tag_pos.y-DISTANCES[i][2])
			rospy.loginfo(err)
			self.pub_error_ar_tag.publish(err)
			self.vel_ctrl()				


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
#			rospy.loginfo("line info received!")
			vx,vy = self.conv_vect(line_params.vx, line_params.vy)
			if vx ==0:
				vx = 0.001
#			print("Velocities:",vx,vy)
			xc,yc =  self.find_closest_point(line_params.x, line_params.y, vx, vy)

#			print("Point closest to line:",xc,yc)
			
			self.ang_err = np.arctan(vy/vx)

			self.pos = self.d_target_position(xc, yc, vx, vy)
			if not (self.isflying_up or self.isflying_forward or self.isflying_reset):
				self.vel_ctrl()
		
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
			rospy.loginfo("Streaming")
			self.vel_ctrl()
			while ((not rospy.is_shutdown()) and self.offboard_point_streaming) or NO_ROBOT:
				
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


if __name__ == "__main__":
	rospy.init_node("line_tracker")
	d = LineTracker()
	rospy.spin()
	d.stop_streaming_offboard_points()
