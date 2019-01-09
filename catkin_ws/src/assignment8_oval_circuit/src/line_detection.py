#!/usr/bin/env python
import sys
import rospy
import matplotlib.pyplot as plt
import numpy as np
import roslib
import sys
import cv2

from sklearn import linear_model
from cv_bridge import CvBridge, CvBridgeError
from collections import namedtuple
from scipy import optimize
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, UInt8, UInt16
from collections import deque

class CaptainSteer:
    def __init__(self):
        self.error_sub = rospy.Subscriber("/image_processing/error", Int16, self.error_callback)
        self.error_queue = deque([], 10)
        self.delta = 5
        self.old = 90
        self.p = 90
        self.k = -3
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)

    def error_callback(self, msg):
        data = msg.data
        # append to list and slowly build to max 10 elements
        self.error_queue.append(data)

        # average over all items
        mean_error = np.mean(list(self.error_queue))
        print("Mean: %s of %s" % (mean_error, self.error_queue))

        steering_angle = (mean_error / self.k) + self.p
        #if (abs(self.old) - abs(steering_angle)) > 1:
        print("Steering Angle: %s %s" % (steering_angle, self.old))
        self.old = steering_angle
        self.pub_steering.publish(UInt8(steering_angle))

class LineDetection:
    def __init__(self, plot):
        self.plot = plot
        if self.plot:
            plt.ion()
            plt.show()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size = 1)
        self.image_black_pub = rospy.Publisher("/image_processing/bin_black_img", Image, queue_size = 1)
        self.error_pub = rospy.Publisher("/image_processing/error", Int16, queue_size = 1)
        self.bridge = CvBridge()


    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        img = np.flip(cv_image, 0)
        #img = img[:350]
        # Convert to grayscale
        #gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Convert to B/W image
        bi_gray_max = 255
        bi_gray_min = 250
        ret, img = cv2.threshold(img, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
        try:
            self.image_black_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
        except CvBridgeError as e:
            print(e)

        X = []
        Y = []
        for y in range(480):
            for x in range(640):
                if img[y, x] == 255:
                    X.append([x])
                    Y.append(y)

        # RANSAC
        ransac = linear_model.RANSACRegressor()
        ransac.fit(X, Y)
        line_x = np.arange(0, 640)[:, np.newaxis]
        line_y = ransac.predict(line_x)

        self.update_plot(X, Y, line_x, line_y)

        #self.error_pub.publish(Int16(320 - x_coord))

    def update_plot(self, X, Y, line_x, line_y):
        x = np.linspace(0, 640, 2)
        plt.clf()
        plt.axis('equal')
        plt.ylim(0,480)
        plt.xlim(0,640)
        plt.plot(X,Y,'ro')
        plt.plot(line_x,line_y,'blue')
        plt.pause(0.001)

def main(args):
    rospy.init_node("oval_circuit")
    cs = CaptainSteer()
    ld = LineDetection(True)
    pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
    pub_speed.publish(Int16(150))

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
