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
from cv_bridge import CvBridge, CvBridgeError
from collections import namedtuple
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, UInt8, UInt16, Float64
from collections import deque
from numpy.linalg import norm

class VelocityController:
    def __init__(self, debug):
        print("INIT")
        self.tick_sub = rospy.Subscriber("/ticks", UInt8, self.callback)
        self.speed_sub = rospy.Subscriber("/image_processing/speed", Float64, self.speed_callback)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
        # Distance per tick in meter
        self.dist_per_tick = 0.005859375
        self.debug = debug
        self.last_tick = None
        self.first_tick = None
        self.desired_speed = None
        self.integral = 0
        self.previous_error = 0
        self.speeds = []
        self.times = []
        # PID Parameters
        self.kp = 0.8
        self.ki = 0.0001
        self.kd = 0.1
        self.pub_speed.publish(Int16(250))
        print("DONE")

    def callback(self, msg):
        # Early exit if no tick
        if msg.data == 0:
            return
        timestamp = time.clock()
        print("TICK CALLBACK")

        # If first tick, save timestamp exit.
        if self.last_tick == None:
            self.last_tick = timestamp
            self.first_tick = timestamp
            return

        speed = self.dist_per_tick / (self.last_tick)

        # PID
        dt = timestamp - self.last_tick
        error = self.desired_speed - speed
        self.integral = self.integral + error * dt
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        self.pub_speed.publish(Int16(output))
        self.last_tick = timestamp
        self.speeds.append(speed)
        self.times.append(timestamp - self.first_tick)

        if self.debug:
            print("Estimated speed: %s m/s" % (speed))
            print("Estimated distance: %s m" % (speed * (timestampe-self.first_tick)))
            print("DT: %s Error: %s Integral: %s Derivative: %s Output: %s" % (dt, error, self.integral, derivative, output))
            print("\nSpeeds: %s\n\n" % self.speeds)
            print("\nTimes: %s\n\n" % self.times)

    def speed_callback(self, msg):
        print("CALLBACK")
        self.desired_speed = msg.data
        self.current_rpm = 200
        self.pub_speed.publish(Int16(200))

    def update_plot(self, X, Y):
        plt.clf()
        plt.ylim(0, 120)
        plt.xlim(0,4)
        plt.plot(X,Y,'ro')
        plt.pause(0.001)


class CaptainSteer:
    def __init__(self, debug):
        self.error_sub = rospy.Subscriber("/image_processing/error", Int16, self.error_callback)
        self.error_queue = deque([], 10)
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
    def __init__(self, plot):
        self.plot = plot
        if self.plot:
            plt.ion()
            plt.show()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size = 1)
        if self.plot:
            self.image_black_pub = rospy.Publisher("/image_processing/bin_black_img", Image, queue_size = 1)
            self.image_gray_pub  = rospy.Publisher("/image_processing/bin_gray_img", Image, queue_size = 1)
        self.error_pub = rospy.Publisher("/image_processing/error", Int16, queue_size = 1)
        self.speed_pub = rospy.Publisher("/image_processing/speed", Float64, queue_size = 1)
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
        bi_gray_min = 220
        ret, img = cv2.threshold(gray_img, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
        if self.plot:
            try:
                self.image_black_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
            except CvBridgeError as e:
                print(e)

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

        self.error_pub.publish(Int16(distance))

    def update_plot(self, X, Y, line_x, line_y):
        x = np.linspace(0, 640, 2)
        plt.clf()
        plt.axis('equal')
        plt.ylim(480,0)
        plt.xlim(0,640)
        plt.plot(X,Y,'ro')
        plt.plot(line_x,line_y,'blue')
        plt.plot([320], [240], 'go')
        plt.pause(0.001)

def main(args):
    rospy.init_node("oval_circuit")
    #cs = CaptainSteer(False)
    #ld = LineDetection(False)
    vc = VelocityController(True)

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
