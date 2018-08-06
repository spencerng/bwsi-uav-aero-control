import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np


_DEBUG = True
class ImageToCV:
	def __init__(self):
		rospy.Subscriber("/aero_downward_camera/image", Image, self.image_cb)
		self.image_pub = rospy.Publisher("/image_to_cv/processed", Image, queue_size=1)
	self.image_pub_gray = rospy.Publisher("/image_to_cv/processed/gray", Image, queue_size=1)
		self.bridge = CvBridge()

	def image_cb(self, msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, "8UC1")
			self.process(cv_image)
		except CvBridgeError as e:
			print(e)

		#cv2.imshow(cv_image) # shows the image, might be VERY slow
		
	def process(self, img):
		laplacian = cv2.Laplacian(img,cv2.CV_64F)
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
		lefty = int((-x*vy/vx) + y)
		righty = int(((cols-x)*vy/vx)+y)
		img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
		cv2.line(img_color,(cols-1,righty),(0,lefty),(255,0,0),2)

		
		if _DEBUG:
			try:
				self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_color, "rgb8"))
			except CvBridgeError as e:
				print(e)

if __name__ == "__main__":
	rospy.init_node("image_to_cv")
	a = ImageToCV()

	#rospy.on_shutdown(cv2.destroyAllWindows())

	rospy.spin()
