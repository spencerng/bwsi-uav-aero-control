#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped, Twist, PoseStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import mavros
from mavros_msgs.msg import State
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from datetime import datetime

VALID_AR_IDS = []
VALIDATE_IDS = False 
AR_FWD_THRESH = 0.75 #desired distance to be away from tag before flying up
AR_FWD_DIST = 0.6 # distance relative to AR tag to fly forward
AR_Z_TOL = 0.2 #tolerance for when drone starts flying forward
AR_Z_DIST = 0.6 #distance to shoot up or down
K_P_Z = 1.5
K_D_Z = 0.0
NO_ROBOT = True

class ARObstacleHandler:
	def __init__(self, rate = 10):
		assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
		self.rate_hz = rate        
		self.rate = rospy.Rate(rate)
		self.ar_pose_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb, queue_size = 1)
		self.current_marker = None
		self.mavros_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.mavros_pose_cb, queue_size = 1)
		self.pub_ar_obstacle_vel = rospy.Publisher("/ar_obstacle/cmd_vel", Float32, queue_size=1)
		self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
		self.current_mode = None
		self.current_pos = None
		self.pub_ar_obstacle_vel.publish(0.0)
		self.prev_z_err = 0.0
		self.current_flying_id = 420
		while True:
			if self.current_marker is not None:
				self.x_err = abs(AR_FWD_THRESH - abs(self.current_marker.pose.pose.position.z))
				self.z_ar_err = -self.current_marker.pose.pose.position.y
				if abs(slef.current_marker.pose.pose.position.z) <= AR_FWD_THRESH:
					self.current_flying_id = self.current_marker.id
					if self.current_marker.id % 2 == 0 and (self.current_mode=="OFFBOARD" or NO_ROBOT):
						self.fly_down()
						self.fly_forward()
						self.fly_up()
					elif (self.current_mode=="OFFBOARD" or NO_ROBOT): 
						self.fly_up()
						self.fly_forward()
						self.fly_down()
				else:
					rospy.loginfo(str(self.x_err) + " m to threshold")
	def state_cb(self, state):
		""" Starts setpoint streamer when mode is "POSCTL" and disables it when mode is "MANUAL"
		:param state: Given by subscribed topic `/mavros/state`
		"""
		self.current_mode = getattr(state, "mode", None)

	def another_ar_tag_close(self):
		if self.current_marker is not None:
			new_x_err = abs(AR_FWD_THRESH - abs(self.current_marker.pose.pose.position.z))
			return (self.current_marker.id != self.current_flying_id and  new_x_err <=AR_FWD_TOL)
		return False

	def fly_up(self):
		z_orig = self.current_pos.z
		z_delta = self.current_pos.z - z_orig
		z_err = (AR_Z_DIST + self.z_ar_err) - z_delta
		prev_time = datetime.now()
		self.prev_z_err = z_err
		while abs(z_err) > AR_Z_TOL and (self.current_mode=="OFFBOARD" or NO_ROBOT) and not self.another_ar_tag_close():
			dt = datetime.now() - prev_time
			dt = dt.total_seconds()
			if dt == 0:
				dt = 0.0001
			rospy.loginfo("Flying up: "+ str(z_err) + " m left")
			z_vel_cmd = -(K_P_Z * z_err+K_D_Z * (z_err - self.prev_z_err)/dt)
			self.pub_ar_obstacle_vel.publish(z_vel_cmd)
			z_delta = self.current_pos.z - z_orig
			prev_time = datetime.now()
			self.prev_z_err = z_err
			z_err = (AR_Z_DIST + self.z_ar_err) - z_delta
			self.rate.sleep()
		self.pub_ar_obstacle_vel.publish(0.0)
		if abs(z_err) <= AR_Z_TOL:
			rospy.loginfo("Nice on!")
		else:
			rospy.loginfo("Nice failure!")
		return

	def fly_down(self):
		z_orig = self.current_pos.z
		z_delta = self.current_pos.z - z_orig
		z_err = (-AR_Z_DIST + self.z_ar_err) - z_delta
		prev_time = datetime.now()
		self.prev_z_err = z_err
		while abs(z_err) > AR_Z_TOL and (self.current_mode=="OFFBOARD" or NO_ROBOT) and not self.another_ar_tag_close():
			dt = datetime.now() - prev_time
			dt = dt.total_seconds()
			if dt == 0:
				dt = 0.0001
			rospy.loginfo("Flying down: " +str(z_err) + " m left")
			z_vel_cmd = -(K_P_Z * z_err+K_D_Z * (z_err - self.prev_z_err)/dt)
			self.pub_ar_obstacle_vel.publish(z_vel_cmd)
			prev_time = datetime.now()
			z_delta = self.current_pos.z - z_orig
			self.prev_z_err = z_err
			z_err = (-AR_Z_DIST + self.z_ar_err) - z_delta
			self.rate.sleep()
		self.pub_ar_obstacle_vel.publish(0.0)
		if abs(z_err) <= AR_Z_TOL:
			rospy.loginfo("Nice on!")
		else:
			rospy.loginfo("Nice failure!")
		return


	def fly_forward(self):
		dist = self.x_err + AR_FWD_DIST #actual distance to fly
		x_orig = self.current_pos.x
		x_delta = self.current_pos.x - x_orig
		x_err = dist - abs(x_delta)
		while abs(x_delta) < AR_FWD_DIST and (self.current_mode=="OFFBOARD" or NO_ROBOT) and not self.another_ar_tag_close():
			rospy.loginfo("Flying forward: " + str(x_err) + " m left")
			x_delta = self.current_pos.x - x_orig
			x_err = dist - abs(x_delta)
		if abs(x_delta) >= AR_FWD_DIST:
			rospy.loginfo("Nice on!")
		else:
			rospy.loginfo("Nice failure!")

	def mavros_pose_cb(self,msg):
		self.current_pos = msg.pose.position
		


	def ar_pose_cb(self, msg):
		markers =  msg.markers
#		rospy.loginfo("Marker data received")
		if len(markers) == 0:
			self.current_marker = None
		else:
			for marker in markers:
				if not VALIDATE_IDS or marker.id in VALID_AR_IDS:
					rospy.loginfo("Valid AR marker received")
			min_dist = markers[0].pose.pose.position.z
			self.current_marker = markers[0]
			for marker in markers:
				current_dist = marker.pose.pose.position.z
				if current_dist < min_dist and marker.id in DISTANCES:
					self.current_marker = marker
					min_dist = current_dist


if __name__ == "__main__":
	rospy.init_node("ar_obstacle_avoid_tracker")
	ctrl = ARObstacleHandler()

	rospy.spin()
