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
        
        left_image = black_img[320:639]
        right_image = black_img[0:319]

        lfoundtop = (0,0)
        lfoundmid = (0,0)
        lfoundbot = (0,0)
        
        rfoundtop = (0,0)
        rfoundmid = (0,0)
        rfoundbot = (0,0)

        #left
        for y in range(480):
            for x in range(320):
                if left_image[x][y] == 255:
                   lfoundtop[0]=x        
                   lfoundtop[1]=y       

        for y in range(480):
            for x in range(320):
                if left_image[x][480-y] == 255:
                   lfoundbot[0]=x        
                   lfoundbot[1]=y       

        for x in range(320):
            if left_image[x][foundbot[1]-foundtop[1]] == 255:
                lfoundmid[0]=x        
                lfoundmid[1]=y       

        #right
        for y in range(480):
            for x in range(320):
                if right_image[x][y] == 255:
                   rfoundtop[0]=x        
                   rfoundtop[1]=y       

        for y in range(480):
            for x in range(320):
                if right_image[x][480-y] == 255:
                   rfoundbot[0]=x        
                   rfoundbot[1]=y       

        for x in range(320):
            if right_image[x][foundbot[1]-foundtop[1]] == 255:
                rfoundmid[0]=x        
                rfoundmid[1]=y

        print(lfoundtop) 
        print(lfoundmid) 
        print(lfoundbot) 
        print(rfoundtop) 
        print(rfoundtop) 
        print(rfoundtop) 

def main(args):
    rospy.init_node('lanesegment', anonymous =True)
    LS = LaneSegment()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    main(sys.argv)

