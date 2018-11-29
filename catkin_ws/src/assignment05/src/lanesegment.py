import roslib
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class LaneSegment:
    def __init__(self):
        # Image publisher
        self.image_gray_pub = rospy.Publisher("/image_processing/bin_gray_img", Image, queue_size = 1)
        self.image_black_pub = rospy.Publisher("/image_processing/bin_black_img", Image, queue_size = 1)
        self.image_pub = rospy.Publisher("/image_processing/bin_img", Image, queue_size = 1)

        # Image source
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size = 1)
        # OpenCV
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # define range of white color in HSV
        lower_white = np.array([180, 180, 180])
        upper_white = np.array([255, 255, 255])
        # threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)
        #bitwise and mask and original image
        res = cv2.bitwise_and(cv_image, cv_image, mask= mask)
        #res = cv2.cvtColor(res, cv2.COLOR_HSV2BGR)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "8UC3"))
        except CvBridgeError as e:
            print(e)

        #cv2.imshow('res', res)


        # Convert to white only
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        try:
            self.image_gray_pub.publish(self.bridge.cv2_to_imgmsg(gray_img, "mono8"))
        except CvBridgeError as e:
            print(e)
        # Convert to white lines only
        bi_gray_max = 255
        bi_gray_min = 250

        ret, black_img = cv2.threshold(gray_img, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
        try:
            self.image_black_pub.publish(self.bridge.cv2_to_imgmsg(black_img, "mono8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('lanesegment', anonymous =True)
    LS = LaneSegment()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)

