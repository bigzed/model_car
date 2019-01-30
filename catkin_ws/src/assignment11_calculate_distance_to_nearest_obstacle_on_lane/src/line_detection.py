#!/usr/bin/env python
import sys
import rospy
import matplotlib.pyplot as plt
import numpy as np
import roslib
import sys
import cv2
import time
import tf
import sensor_msgs.point_cloud2 as pc2

from sklearn import linear_model
from scipy.misc import imread
from cv_bridge import CvBridge, CvBridgeError
from collections import namedtuple
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Int16, UInt8, UInt16, Float64
from laser_geometry import LaserProjection
from collections import deque
from nav_msgs.msg import Odometry
from numpy.linalg import norm
from tf.transformations import *

class TFBroadcaster:
    def __init__(self, car_id):
        """Subscribe to odometry for car_id"""
        self.local_sub = rospy.Subscriber("/localization/odom/" + str(car_id), Odometry, self.get_odom, queue_size=1)

    def get_odom(self, msg):
        """For each package send new transformation"""
        br = tf.TransformBroadcaster()
        q = quaternion_from_euler(0, 0, np.pi)
        q_odom = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        q2 = quaternion_multiply(q_odom, q)
        br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                (q2[0], q2[1], q2[2], q2[3]),
                msg.header.stamp, 'laser', 'map')

class LazerHawk:
    def __init__(self):
        """Subscribe to LIDAR"""
        # Get Lidar
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        # Pub Pointcloud
        self.point_cloud_pub = rospy.Publisher("/localization/point_cloud", PointCloud2, queue_size=1)
        self.lp = LaserProjection()

    def lidar_callback(self, msg):
        """Subscribe to LIDAR and publish point_cloud"""
        point_cloud = self.lp.projectLaser(msg)
        self.point_cloud_pub.publish(point_cloud)

class VelocityController:
    def __init__(self, debug):
        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
        self.curr_speed_pub = rospy.Publisher("/curr_speed", Float64, queue_size=1)
        self.td_pub = rospy.Publisher("/time_diff", Float64, queue_size=1)
        self.debug = debug
        self.shutdown = 0
        rospy.on_shutdown(self.shutdownhandler)
        self.previous_error = 0
        self.integral = 0
        self.desired_speed = 0
        self.last_ts = 0
        self.tickq = deque([0]*10)
        # PID parameters
        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.11

        # Subscribers
        self.tick_sub = rospy.Subscriber("/ticks", UInt8, self.get_ticks)
        self.speed_sub = rospy.Subscriber("/desired_speed", Float64, self.get_speed)
        self.speed_sub = rospy.Subscriber("/time_diff", Float64, self.adapt_speed)

    def shutdownhandler(self):
        self.shutdown = .2
        self.speed_pub.publish(Int16(0))

    def get_ticks(self, msg):
        self.tickq.popleft()
        self.tickq.append(msg.data)
        tickavg = np.mean(list(self.tickq)) * 0.058
        self.td_pub.publish(tickavg)

    def get_speed(self, msg):
        self.desired_speed = msg.data

    def adapt_speed(self, msg):
        # Calculate speed estimate in m/s
        self.curr_speed_pub.publish(Float64(msg.data))

        # PID Controller
        error = self.desired_speed - msg.data
        self.integral = self.integral + error * 0.1
        derivative = (error - self.previous_error) / 0.1

        # PID Controller operates in m/s and is scaled to ticks/s
        speed_in_rpm = (self.kp * error + self.ki * self.integral + self.kd * derivative) * 170
        if self.shutdown == 0:
            self.speed_pub.publish(Int16(speed_in_rpm))

class CaptainSteer:
    def __init__(self, debug):
        self.error_queue = deque([], 1)
        self.delta = 5
        self.old = 90
        self.p = 90
        self.k = -3
        self.debug = debug

        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)
        self.sub_steering = rospy.Subscriber("/steering", UInt8, self.get_steering)
        self.pub_speed = rospy.Publisher("/desired_speed", Float64, queue_size=100, latch=True)

        #TODO error should be calculated from the pose:
        self.error_sub = rospy.Subscriber("/localization/error", Int16, self.error_callback)
        #self.error_sub = rospy.Subscriber("/image_processing/error", Int16, self.error_callback)

    def get_steering(self, msg):
        if msg.data < 50 or msg.data > 130:
            self.pub_speed.publish(Float64(0.8))
        else:
            self.pub_speed.publish(Float64(1.2))

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
        self.last_dist = 320
        self.bridge = CvBridge()

        if self.plot:
            self.image_black_pub = rospy.Publisher("/image_processing/bin_black_img", Image, queue_size = 1)
            self.image_gray_pub  = rospy.Publisher("/image_processing/bin_gray_img", Image, queue_size = 1)
        self.error_pub = rospy.Publisher("/image_processing/error", Int16, queue_size = 1)
        self.error_pub.publish(Int16(120))

        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback, queue_size = 1)

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

        #distance = self.ransac_distance_to_center(img)
        distance = self.naive_distance_to_center(img)
        #distance = self.naive_distance_average(img)

        self.error_pub.publish(Int16(distance))

    def naive_distance_to_center(self, img):
        # Get all white points in row 240
        coords = np.where(img == 255)
        dist_change = False
        for x in range(320):
            if x < 140 and img[300, 320 + x] == 255:
                dist = 320 + x
                dist_change=True
                break
            if img[300, 320 - x] == 255:
                dist = 320 - x
                dist_change=True
                break

        if dist_change == False:
            dist = self.last_dist
        self.last_dist = dist

        if self.plot:
            print("Error: %s" % (320 - dist))
            self.update_plot(coords[1], coords[0], [dist], [300])

        return 320 - dist

    def naive_distance_average(self, img):
        # Average over line 300
        avg = 0
        avg_cnt = 0
        for y in range(0,640):
            if img[300,y] == 255:
                avg += y
                avg_cnt += 1
        if avg_cnt > 0:
            dist = avg / avg_cnt
            self.last_dist = dist
        else:
            dist = int((320 * 0.2) + ((self.last_dist - 320) * 0.8))

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
    def __init__(self, car_id, debug = False):
        self.error_pub = rospy.Publisher("/localization/error", Int16, queue_size=1)
        self.speed_pub = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
        self.local_sub = rospy.Subscriber("/localization/odom/" + str(car_id), Odometry, self.localization_callback, queue_size=1)
        self.point_cloud_sub = rospy.Subscriber("/localization/point_cloud", PointCloud2, self.get_point_cloud, queue_size=1)
        self.positions = []
        self.point_cloud = None
        self.tf = tf.TransformListener()
        self.debug = debug
        self.lane_id = 1

    def get_point_cloud(self, msg):
        """Save current LIDAR PointCloud in Car coordinates"""
        self.point_cloud = msg

    def lane_is_free(self, obstacles, lane_id):
        for p in obstacles:
            point = np.array(p)
            if abs(np.linalg.norm(point - self.closest_point(point, lane_id, 0))) < 16:
                return False

        return True

    def get_lane(self, car_x, car_y):
        """Returns LaneID based on obstacles on the road."""

        """Transform and filter for obstacles in 1.5m distance"""
        obstacles = []
        for p in pc2.read_points(self.point_cloud):
            ps = PointStamped(self.point_cloud.header, Point(p[0], p[1], p[2]))
            new_ps = self.tf.transformPoint('map', ps)
            """Our coordinates are in CM and switched because of the image plotting"""
            if abs(np.linalg.norm(np.array([car_x, car_y]) - np.array([new_ps.point.y * 100, new_ps.point.x * 100]))) < 150:
                obstacles.append([new_ps.point.y * 100, new_ps.point.x * 100])

        """Plot detected obstacles"""
        if self.debug:
            self.obstacles_x, self.obstacles_y = np.array(obstacles).T

        """Check if obstacle is on either lane"""
        if self.lane_is_free(obstacles, self.lane_id):
            """STAY"""
        elif self.lane_is_free(obstacles, (self.lane_id + 1) % 2):
            """SWITCH"""
            self.lane_id = (self.lane_id + 1) % 2
        else:
            """STOP"""
            self.speed_pub.publish(Int16(0))

        return self.lane_id

    def localization_callback(self, msg):
        """Return if we have no data yet"""
        if not self.point_cloud:
            return
        if not self.tf.canTransform('map', self.point_cloud.header.frame_id, self.point_cloud.header.stamp):
            return

        """Odometry coordinates are in M ours are in CM and switched because of the image plotting"""
        self.car_x = msg.pose.pose.position.y * 100
        self.car_y = msg.pose.pose.position.x * 100

        """Get lane_id based on obstacles"""
        self.lane_id = self.get_lane(self.car_x, self.car_y)

        """Get next point on trajectory use to determine steering error"""
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        carangle = np.degrees(yaw) # transform to degrees and shift from -180/180 to 0/360
        self.front_vec = np.array([np.sin(yaw), np.cos(yaw)]) * 10
        car = self.front_vec + np.array([self.car_x, self.car_y])
        #print(car, yaw)

        self.distpoint = self.closest_point([self.car_x, self.car_y], self.lane_id, 15)
        """ Why distpoint[-1]?"""
        self.target = np.linalg.norm(self.distpoint[-1]) - np.array([self.car_x, self.car_y])

        errorangle = np.degrees(np.arccos(np.dot(self.target, car) / (np.linalg.norm(self.target) * np.linalg.norm(car))))
        error = errorangle * 2
        #print("distpoint[-1], [x,y], car, target, carangle, errorangle, error")
        #print(carangle, errorangle, error)

        self.plot()
        self.error_pub.publish(Int16(error))

    def closest_point(self, point, laneID, distance):
        x,y = point[0], point[1]

        if laneID == 1: #INNERLANE
            laned = 16
        elif laneID == 2: #OUTERLANE
            laned = 48

        """Returns closest point on trajectory."""
        # Top or bottom center of circle
        if x == 215 and y == 196:
            return np.array([215, 76 + laned])
        if x == 215 and y == 404:
            return np.array([215, 524 + laned])

        # Top half circle:
        if y <= 196:
            return self.dist_on_circle(np.array([x, y]), np.array([215, 196]), 121 + laned, distance)
        # Bottom half circle:
        elif y >= 404:
            return self.dist_on_circle(np.array([x, y]), np.array([215, 404]), 121 + laned, distance)
        # Left line
        elif x <= 215:
            if y + distance >= 404:
                return self.closest_point_on_circle(np.array([x, y - distance]), np.array([215, 196]), 121 + laned)
            return np.array([94 - laned, y + distance])
        # Right line
        else:
            if y - distance >= 196:
                return self.closest_point_on_circle(np.array([x, y - distance]), np.array([215, 404]), 121 + laned)
            return np.array([336 + laned, y - distance])

    def closest_point_on_circle(self, p, c, r):
        v = p - c
        return c + v / np.linalg.norm(v) * r

    def dist_on_circle(self, p, c, r, dist):
        v = p - c
        distpoint = c + (v * r / np.linalg.norm(v))
        U = 2 * np.pi * r
        distanceangle = 360 * (dist / U)
        c, s = np.cos(distanceangle), np.sin(distanceangle)
        distpoint[0] = distpoint[0]*c + distpoint[1]*(-s)
        distpoint[1] = distpoint[0]*s + distpoint[1]*(c)
        return distpoint

    def plot(self):
        img = imread('texinput/pictures/map.png')
        plt.ion()
        plt.clf()
        plt.imshow(img)
        plt.plot(self.car_x, self.car_y, 'gx')
        plt.plot(self.obstacles_x, self.obstacles_y, 'y.')
        plt.plot(self.distpoint[0], self.distpoint[1], 'bx')
        plt.plot(self.target[0], self.target[1], 'rx')
        plt.axes().arrow(self.car_x, self.car_y, self.front_vec[0], self.front_vec[1], head_width=4, head_length=6, color='w')
        plt.show()
        plt.pause(0.0001)

def main(args):
    rospy.init_node("oval_circuit")
    car_id  = 3
    #cs = CaptainSteer(False)
    tf = TFBroadcaster(car_id)
    lh = LazerHawk()
    #ld = LineDetection(False)
    localization = Localization(car_id, True)
    #vc = VelocityController(False)

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
