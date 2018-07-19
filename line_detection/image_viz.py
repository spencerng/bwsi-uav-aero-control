import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image


_DEBUG = True
class ImageToCV:
    def __init__(self):
        rospy.Subscriber("/aero_downward_camera/image", Image, self.image_cb)
        self.image_pub = rospy.Publisher("/image_to_cv/processed", Image, queue_size=1)
        self.bridge = CvBridge()
        rospy.init_node('talker', anonymous=True)

    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError as e:
            print(e)

        #cv2.imshow(cv_image) # shows the image, might be VERY slow (optional)
        self.process(cv_image)

    def liner(s,img_s,r):
    
        unmod_data = np.argwhere(img_s)
        data = np.argwhere(s)
    
        x = data[:, 1]
        y = data[:, 0]
    
        X = np.column_stack((x, np.ones_like(x)))

        xx = np.arange(0, s.shape[0])

        model = LineModelND()
        model.estimate(data)
        model.params
        origin, direction = model.params
        model_robust, inliers = ransac(data, LineModelND, min_samples=2,
                                    residual_threshold=r, max_trials=1000)
        outliers = (inliers == False)
        yy = model_robust.predict_y(xx)
        return (yy,xx)




    def img_cleanup(img):
        img_s = img*3
        maskr = cv2.inRange(img_s, 240, 255)
        s = cv2.bitwise_and(img_s,img_s, mask= maskr)
        kernel = np.ones((2,2),np.uint8)
        kernel1 = np.ones((3,3),np.uint8)
        s = cv2.dilate(s, kernel, iterations = 1)
        s = cv2.erode(s, kernel, iterations = 2)
        return liner(s,img,5)

    def process(self, img):
        yy,xx = img_cleanup(img)
        rospy.loginfo("yy: "+yy+" xx: "+xx, topic)

        if _DEBUG:
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))

            except CvBridgeError as e:
                print(e)

if __name__ == "__main__":
    rospy.init_node("image_to_cv")
    a = ImageToCV()

    rospy.on_shutdown(cv2.destroyAllWindows())

    rospy.spin()
