import roslib
import sys
import rospy
import cv2
import sklearn
import numpy as np

from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class LaneSegment:
    def __init__(self):
        # Image publisher
        self.image_gray_pub = rospy.Publisher("/image_processing/bin_gray_img", Image, queue_size = 1)
        self.image_black_pub = rospy.Publisher("/image_processing/bin_black_img", Image, queue_size = 1)
        self.ransac = rospy.Publisher("/image_processing/bin_ransac_lines", Image, queue_size = 1)

        # Image source
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size = 1)
        # OpenCV
        self.bridge = CvBridge()

    def callback(self, data):

        # +++ Exercise 1 +++
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

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
        
        # +++ Exercise 2 +++

        left_image = black_img[0:319]
        right_image = black_img[320:639]

        points_left = ([][])*3
        points_right = ([][])*3

        # left
        # search from top to bottom to find a starting white point
        for y in range(480):
            for x in range(320):
                if left_image[x][y] == 255:
                   points_left[0][0]=x        
                   points_left[0][1]=y       

        # search from bottom to top to find an ending white point
        for y in range(480):
            for x in range(320):
                if left_image[x][480-y] == 255:
                   points_left[1][0]=x        
                   points_left[1][1]=y       

        # search in the middle of both white points to find a corresponding white point
        for x in range(320):
            if left_image[x][points_left[1][0]-points_left[2][0] == 255:
               points_left[2][0]=x        
               points_left[2][1]=y       

        # right
        # analogous for the right pixels
        for y in range(480):
            for x in range(320):
                if right_image[x][y] == 255:
                   points_right[0][0]=x        
                   points_right[0][1]=y       

        for y in range(480):
            for x in range(320):
                if right_image[x][480-y] == 255:
                   points_right[1][0]=x        
                   points_right[1][1]=y       

        for x in range(320):
            if right_image[x][foundbot[1]-foundtop[1]] == 255:
               points_right[2][0]=x        
               points_right[2][1]=y       


        ransac[0] = sklearn.linear_model.RANSACRegressor(min_samples=3, max_trials=1)
        ransac[1] = sklearn.linear_model.RANSACRegressor(min_samples=3, max_trials=1)

        try:
            self.image_black_pub.publish(ransac)
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

