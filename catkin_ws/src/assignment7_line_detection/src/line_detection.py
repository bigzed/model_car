#!/usr/bin/env python
import sys
import rospy
import matplotlib.pyplot as plt
import numpy as np
import roslib
import sys
import cv2

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
        self.inlier_dist = 0.05
        self.sample_count = 50
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
        img = img[:350]
        # Convert to grayscale
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Convert to B/W image
        bi_gray_max = 255
        bi_gray_min = 215
        ret, img = cv2.threshold(gray_img, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
        try:
            self.image_black_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
        except CvBridgeError as e:
            print(e)

        x_coord = 0
        for x in range(320):
            if img[240, 320 + x] == 255:
                x_coord = 320 + x
                break
            if img[240, 320 - x] == 255:
                x_coord = 320 - x
                break
        self.error_pub.publish(Int16(320 - x_coord))  

    def calculate_error(self, slope, intercept):
        if slope == 0:
            return 0
        return (320 - (240 - intercept) / slope)

    def find_line(self, points):
	""" find the best params to describe a detected line using the RANSAC algorithm """
	best_count = 0
	best_params = (0,0)
	xs, ys = points.T

	# randomly sample points to define a line and remember the one with the most inliers
	for _ in xrange(self.sample_count):
            if (len(xs) == 0):
                print("warn: The wall couldn't be found!")
                continue

            ind = np.random.randint(0, len(xs), 2)
            x_a, x_b = xs[ind]
            if x_a == x_b:
                continue  # avoid division by 0

            y_a, y_b = ys[ind].astype(np.float64)

            slope = (y_b - y_a) / (x_b - x_a)
            intercept = y_a - x_a * slope

            inlier = self.get_inliers(points, slope, intercept)
            inl_count = np.sum(inlier)
            if inl_count > best_count:
                best_count = inl_count
                best_params = (slope, intercept)

	# the set of points within inlier distance of the best line we found
	inlier_points = points[np.where(self.get_inliers(points, *best_params))]

        return best_params

    def get_inliers(self, points, slope, intercept):
            """ return a numpy boolean array for each point (True if within 'inlier_dist' of the line, else False). """
            return self.get_distance(points, slope, intercept) <= self.inlier_dist

    def get_distance(self, points, slope, intercept):
            """ return the distance for each point to the parametrised line """
            pos = np.array((0, intercept))  # origin
            dir = np.array((1, slope))  # line gradient
            # element-wise cross product of each points origin offset with the gradient
            c = np.cross(dir, pos - points, axisb=-1)
            return np.abs(c) / np.linalg.norm(dir)

    def update_plot(self, X, Y, error, slope, intercept):
        """ update the plot with new lidar data. """
        x = np.linspace(0, 640, 2)
        plt.clf()
        plt.axis('equal')
        plt.ylim(0,480)
        plt.xlim(0,640)
        plt.plot([error],[240],'green', markersize = 14)
        plt.plot(X,Y,'ro')
        plt.plot(x,(slope*x + intercept), color='blue')
        plt.pause(0.001)

def main(args):
    rospy.init_node("line_detection")
    cs = CaptainSteer()
    ld = LineDetection(False)
    pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
    pub_speed.publish(Int16(150))

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
