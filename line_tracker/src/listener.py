#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from aero_control.msg import Line


'''
A python script to practice receiving ROS messages
'''

class Listener():
	''' Subscribes to ROS messages
	'''
	def __init__(self):
		rospy.Subscriber("/line_detection/line", Line, self.chatter_callback)# raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")

	def chatter_callback(self, msg):
		''' Function to be run everytime a message is received on chatter topic
		'''
		rospy.loginfo(msg)


if __name__ == '__main__':

	rospy.init_node('listener')
	l_obj = Listener()
	print("Listener node running")
	rospy.spin()
