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

		#raise Exception("CODE INCOMPLETE! Delete this exception and complete the following lines")
		self.sub_cam = rospy.Subscriber("/aero_downward_camera/image", Image, self.image_cb)
		#raise Exception("CODE INCOMPLETE! Delete this exception and complete the following lines")
		# self.sub_cam = # TODO: subscribe to downward facing camera, and set callaback to image_cb
		# self.pub_param = # TODO: create a publisher for line parameterizations

		
		self.pub_param = rospy.Publisher('/line_detection/line',Line, queue_size=1)
		
		self.image_pub = rospy.Publisher('/line_detection/feed',Image, queue_size=1)
		self.bridge = CvBridge()
		#raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
		# TODO-END

	def parameterizeLine(self, img):

		""" Fit a line to LED strip
		:param cv_image: opencv image
		"""
		# TODO-START: run linear regression to parameterize the line

		result = cv2.inRange(img,245,255)
		kernel=np.ones((3,3),np.uint8)    
		result = cv2.dilate(result, kernel, iterations=2)
		kernel=np.ones((3,3),np.uint8)
		result = cv2.erode(result, kernel, iterations=2)
	  
	
		ret,thresh = cv2.threshold(result,245,255,cv2.THRESH_BINARY)
		_, contours, _ = cv2.findContours(thresh, 1, 2)
		if len(contours)==0:
			return

		cnt = contours[0]
		max_area = 0
		for cont in contours:
			if cv2.contourArea(cont) > max_area:
				cnt = cont
				max_area = cv2.contourArea(cont)
			rows,cols = img.shape[:2]
	
		[vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
		if vx[0] < 0.00001:
			vx = 0.00001
	
		
		

		#raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
		# TODO-END
		
		if not _DEBUG:
			try:
				lefty = int((-x*vy/vx) + y)
				righty = int(((cols-x)*vy/vx)+y)
				img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
				cv2.line(img_color,(cols-1,righty),(0,lefty),(255,0,0),2)
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_color, "rgb8"))
				print (x,y,vx,vy)
			except CvBridgeError as e:
				print(e)
		return (x,y,vx,vy)
		# TODO-START: return x, y, vx, and vy in that order
		#raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
		# TODO-END

	def image_cb(self, data):
		
		line = self.parameterizeLine(self.bridge.imgmsg_to_cv2(data, "8UC1"))
		
		print(line)
		if line is not None:
			print("Imgage sending")
			rospy.loginfo("Message sending")
			x, y, vx, vy = line
			msg = Line()
			msg.x=x
			msg.y=y
			msg.vx=vx
			msg.vy=vy
			# TODO-START: create a line message and publish it with x, y, vx, and vy
			self.pub_param.publish(msg)
			#raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
			# TODO-END
			

if __name__ == '__main__':
	'''
	This is where the code starts running
	'''
	rospy.init_node('line_detector', anonymous=True)

	ld = LineDetector()
	rospy.loginfo("Line detector initialized")
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	except Exception as e:
		print(e)
