#!/usr/bin/env python
import sys
import rospy
import matplotlib.pyplot as plt
import numpy as np
import roslib
import sys
import cv2
import time

from sklearn import linear_model
from scipy.misc import imread
from cv_bridge import CvBridge, CvBridgeError
from collections import namedtuple
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, UInt8, UInt16, Float64
from collections import deque
from nav_msgs.msg import Odometry
from numpy.linalg import norm

class VelocityController:
    def __init__(self, debug):
        self.debug = debug
        # Subscribers
        self.tick_sub = rospy.Subscriber("/ticks", UInt8, self.get_ticks)
        self.speed_sub = rospy.Subscriber("/desired_speed", Float64, self.get_speed)
        self.speed_pub = rospy.Publisher("/speed", Int16, queue_size=100, latch=True)
        self.curr_speed_pub = rospy.Publisher("/curr_speed", Float64, queue_size=1)

        self.total_ticks = 0
        self.last_ticks = 0
        self.previous_error = 0
        self.integral = 0
        self.desired_speed = 0
        # Distance in meter traveled per tick
        self.dist_per_tick = 0.005859375
        # Sleep time in loop
        self.time_slice = 0.1
        # PID parameters
        self.kp = 0.6
        self.ki = 0.1
        self.kd = 0.1

    def get_ticks(self, msg):
        if msg.data == 1:
            self.total_ticks += 1

    def read_ticks(self):
        return self.total_ticks - self.last_ticks

    def get_speed(self, data):
        self.desired_speed = msg.data

    def run(self):
        while not rospy.is_shutdown():
            # One read on self.total_ticks always guarentees to be atomic.
            curr_ticks = self.read_ticks()
            # if total_ticks clips, reset last_ticks
            if curr_ticks < 0:
                self.last_ticks = 0
                curr_ticks = self.read_ticks()
            self.last_ticks += curr_ticks

            # Calculate speed estimate in m/s
            curr_speed = (curr_ticks * self.dist_per_tick) / self.time_slice
            self.curr_speed_pub.publish(Float64(curr_speed))

            # PID Controller
            error = self.desired_speed - curr_speed
            self.integral = self.integral + error * self.time_slice
            derivative = (error - self.previous_error) / self.time_slice

            # PID Controller operates in m/s and is scaled to ticks/s
            speed_in_rpm = (self.kp * error + self.ki * self.integral + self.kd * derivative) * 170
            self.speed_pub.publish(Int16(speed_in_rpm))

            # Sleep for time_slice
            rospy.sleep(self.time_slice)

class CaptainSteer:
    def __init__(self, debug):
        self.error_sub = rospy.Subscriber("/image_processing/error", Int16, self.error_callback)
        self.error_queue = deque([], 1)
        self.delta = 5
        self.old = 90
        self.p = 90
        self.k = -3
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)
        self.debug = debug

    def error_callback(self, msg):
        data = msg.data
        # append to list and slowly build to max 10 elements
        self.error_queue.append(data)

        # average over all items
        mean_error = np.mean(list(self.error_queue))
        if self.debug:
            print("Mean: %s of %s" % (mean_error, self.error_queue))

        steering_angle = (mean_error / self.k) + self.p
        if self.debug:
            print("Steering Angle: %s %s" % (steering_angle, self.old))
        self.old = steering_angle
        self.pub_steering.publish(UInt8(steering_angle))

class LineDetection:
    def __init__(self, plot, ransac = False):
        self.plot = plot
        self.ransac = ransac
        if self.plot:
            plt.ion()
            plt.show()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size = 1)
        if self.plot:
            self.image_black_pub = rospy.Publisher("/image_processing/bin_black_img", Image, queue_size = 1)
            self.image_gray_pub  = rospy.Publisher("/image_processing/bin_gray_img", Image, queue_size = 1)
        self.error_pub = rospy.Publisher("/image_processing/error", Int16, queue_size = 1)
        self.speed_pub = rospy.Publisher("/image_processing/speed", Float64, queue_size = 1)
        self.last_dist = 320
        self.bridge = CvBridge()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = cv_image[:350]
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        if self.plot:
            try:
                self.image_gray_pub.publish(self.bridge.cv2_to_imgmsg(gray_img, "mono8"))
            except CvBridgeError as e:
                print(e)

        # Convert to B/W image
        bi_gray_max = 255
        bi_gray_min = 200
        ret, img = cv2.threshold(gray_img, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
        if self.plot:
            try:
                self.image_black_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
            except CvBridgeError as e:
                print(e)

        if self.ransac:
            distance = self.ransac_distance_to_center(img)
        else:
            distance = self.naive_distance_to_center(img)

        self.error_pub.publish(Int16(distance))

    def naive_distance_to_center(self, img):
        # Get all white points in row 240
        coords = np.where(img == 255)
        dist = -1
        for x in range(320):
            if x < 140 and img[300, 320 + x] == 255:
                dist = 320 + x
                break
            if img[300, 320 - x] == 255:
                dist = 320 - x
                break

        if dist == -1:
            dist = self.last_dist
        self.last_dist = dist

        if self.plot:
            print("Error: %s" % (320 - dist))
            self.update_plot(coords[1], coords[0], [dist], [300])

        return 320 - dist

    def ransac_distance_to_center(self, img):
        # RANSAC
        coords = np.where(img == 255)
        ransac = linear_model.RANSACRegressor()
        ransac.fit(coords[1].reshape(-1, 1), coords[0])
        line_x = np.arange(0, 640)[:, np.newaxis]
        line_y = ransac.predict(line_x)

        # Distance to image center
        center_p = np.array([320, 240])
        line_p1 = np.array([10, line_y[10]])
        line_p2 = np.array([200, line_y[200]])
        distance = np.cross(line_p2-line_p1, line_p1-center_p)/norm(line_p2-line_p1)

        if self.plot:
            print("Error: %s" % (distance))
            self.update_plot(coords[1], coords[0], line_x, line_y)

        return distance

    def update_plot(self, X, Y, line_x, line_y):
        x = np.linspace(0, 640, 2)
        plt.clf()
        plt.axis('equal')
        plt.ylim(480,0)
        plt.xlim(0,640)
        plt.plot(X,Y,'ro')
        if len(line_x) == 1:
            plt.plot(line_x, line_y,'bo')
        else:
            plt.plot(line_x, line_y,'b')
        plt.plot([320], [300], 'go')
        plt.pause(0.001)

class Localization:
    def __init__(self):
        self.local_sub = rospy.Subscriber("/localization/odom/9", Odometry, self.localization_callback, queue_size = 1)
        self.positions = []

    def localization_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.positions.append([y * 100, x * 100])

    def closest_point(self, x, y):
        """Returns closest point on trajectory."""
        # Top or bottom center of circle
        if x == 215 and y == 196:
            return np.array([215, 76])
        if x == 215 and y == 404:
            return np.array([215, 524])

        # Top half circle:
        if y <= 196:
            return self.closest_point_on_circle(np.array([x, y]), np.array([215, 196]), 121)
        # Bottom half circle:
        elif y >= 404:
            return self.closest_point_on_circle(np.array([x, y]), np.array([215, 404]), 121)
        # Left line
        elif x <= 215:
            return np.array([94, y])
        # Right line
        else:
            return np.array([336, y])

    def closest_point_on_circle(self, p, c, r):
        v = p - c
        return c + v / np.linalg.norm(v) * r

    def plot(self):
        # Error
        np_positions = np.array(self.positions)
        cl_points = np.array(map(lambda x: self.closest_point(x[0], x[1]), np_positions))

        dists = map(lambda x: np.absolute(np.linalg.norm(np.array([x[0], x[1]]) - self.closest_point(x[0], x[1]))), np_positions)
        print("mean absolute Error: %s" % (np.mean(dists)))
        print("mean squared Error: %s" % (np.mean(np.square(dists))))

        img = imread('texinput/pictures/map.png')
        plt.ion()
        plt.imshow(img)
        x, y = np_positions.T
        plt.plot(x, y, 'g')
        plt.show(block=True)


def main(args):
    rospy.init_node("oval_circuit")
    cs = CaptainSteer(False)
    ld = LineDetection(False)
    localization = Localization()
    #vc = VelocityController(False)
    #vc.run()

    rospy.spin()

    localization.plot()


if __name__ == '__main__':
    main(sys.argv)
