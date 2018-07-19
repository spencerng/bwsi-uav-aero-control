#!/usr/bin/env python

from __future__ import division, print_function

import rospy
# TODO-START: Import our custom message
from aero_control.msg import Line #raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
# TODO-END


'''
A python script to practice receiving ROS messages
'''

class Listener():
    ''' Subscribes to ROS messages
    '''
    def __init__(self):
        # TODO-START: subscribe to our custom chatter topic, using chatter_callback as the callback
        rospy.init_node('custom_listener', anonymous=True)
        rospy.Subscriber("/custom_chatter", Line, callback)# raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        # TODO-END

    def chatter_callback(self, msg):
        ''' Function to be run everytime a message is received on chatter topic
        '''
        # TODO-START: print the x, y, vx, and vy values in msg
        rospy.log(msg) #raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        # TODO-END

if __name__ == '__main__':
    '''
    This is where the code starts running
    '''
    rospy.init_node('listener')
    l_obj = Listener()
    print("Listener node running")
    rospy.spin()
