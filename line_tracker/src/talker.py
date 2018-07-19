#!/usr/bin/env python

from __future__ import division, print_function

import rospy
# TODO-START: Import our custom message
from line_tracker.msg import Line
#raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
# TODO-END

'''
A python script to practice sending ROS messages
'''

class Talker():
    ''' Generates and publishes ROS messages
    '''
    def __init__(self, chat_frequency=1.0):

        
        #raise Exception("CODE INCOMPLETE! Delete this exception and complete the following lines")
        self.chatter_pub = rospy.Publisher('/custom_chatter',Line) # TODO subscribe to our custom chatter topic, using chatter_callback as the callback

        # rate of publishing
        self.chat_frequency = rospy.Rate(chat_frequency)
	msg = Line()
	msg.x = 0
	msg.y = 0
	msg.vx = 0
	msg.vy = 0
		
    def start_chatter(self):
        ''' send messages on chatter topic at regular rate
        '''
        i = 0
        while (not rospy.is_shutdown()):
            i = i + 1
            # TODO-START: create and publish a custom message [values can be anything]
            self.chatter_pub.publish(msg)#raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
            # TODO-END
            
            self.chat_frequency.sleep()

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('talker')
    td = Talker()
    print("Talker node running")

    # start the chatter
    td.start_chatter()
    rospy.spin()
