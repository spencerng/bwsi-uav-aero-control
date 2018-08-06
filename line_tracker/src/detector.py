#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aero_control.msg import Line
import sys

_DEBUG = True

class LineDetector:
	def __init__(self):

		self.sub_cam = rospy.Subscriber("/aero_downward_camera/image", Image, self.image_cb)
		self.pub_param = rospy.Publisher('/line_detection/line',Line, queue_size=1)
		self.image_pub = rospy.Publisher('/line_detection/feed',Image, queue_size=1)
		
		self.bridge = CvBridge()

	# Creates a CV2 parameterization of a line in img
	def parameterizeLine(self, img):

		""" Fit a line to LED strip
		:param cv_image: opencv image
		"""

		result = cv2.inRange(img,245,255)
		
		kernel=np.ones((3,3),np.uint8)    
		result = cv2.dilate(result, kernel, iterations=2)
		kernel=np.ones((3,3),np.uint8)
		result = cv2.erode(result, kernel, iterations=2)
	  
		#After opening, remove all pixels with intensities less than 245
		ret,thresh = cv2.threshold(result,245,255,cv2.THRESH_BINARY)
		_, contours, _ = cv2.findContours(thresh, 1, 2)
		if len(contours)==0:
			return

		largest_cont = contours[0]
		max_area = 0

		#Find the largest contour to run fitLine on
		for cont in contours:
			if cv2.contourArea(cont) > max_area:
				largest_cont = cont
				max_area = cv2.contourArea(cont)
			rows,cols = img.shape[:2]
	
		[vx,vy,x,y] = cv2.fitLine(largest_cont, cv2.DIST_L2,0,0.01,0.01)
		
		if _DEBUG: #Publishes the image to a ROS topic
			try:
				lefty = int((-x*vy/vx) + y)
				righty = int(((cols-x)*vy/vx)+y)
				img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
				cv2.line(img_color,(cols-1,righty),(0,lefty),(255,0,0),2)
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_color, "rgb8"))
			except CvBridgeError as e:
				print(e)
		return (x,y,vx,vy)

	def image_cb(self, data):
		
		line = self.parameterizeLine(self.bridge.imgmsg_to_cv2(data, "8UC1"))
		
		if line is not None:
			x, y, vx, vy = line
			msg = Line()
			msg.x=x
			msg.y=y
			msg.vx=vx
			msg.vy=vy
			
			self.pub_param.publish(msg)
			
			

if __name__ == '__main__':

	rospy.init_node('line_detector', anonymous=True)

	ld = LineDetector()
	rospy.loginfo("Line detector initialized")
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	except Exception as e:
		print(e)
