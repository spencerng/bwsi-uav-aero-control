#!/usr/bin/env python

from __future__ import division, print_function

import rospy
from aero_control.msg import Line

'''
A python script to practice sending ROS messages
'''

class Talker():
	''' Generates and publishes ROS messages
	'''
	def __init__(self, chat_frequency=1.0):

		self.chatter_pub = rospy.Publisher('/custom_chatter',Line)

		# rate of publishing
		self.chat_frequency = rospy.Rate(chat_frequency)
		
	def start_chatter(self):
		''' send messages on chatter topic at regular rate
		'''
		i = 0
		while (not rospy.is_shutdown()):
			i = i + 1
			msg = Line()
			msg.x = i
			msg.y = i+1
			msg.vx = i+2
			msg.vy = i+3
			self.chatter_pub.publish(msg)
			self.chat_frequency.sleep()

if __name__ == '__main__':
	rospy.init_node('talker')
	td = Talker()
	print("Talker node running")

	# start the chatter
	td.start_chatter()
	rospy.spin()
