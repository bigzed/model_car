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
        yaw = tf.transformations.euler_from_quaternion(q_odom)[2]
        """Move point by 10cm back to correct LIDAR position."""
        lidar_vec = np.array([np.cos(yaw), np.sin(yaw)]) * 0.24
        br.sendTransform((msg.pose.pose.position.x + lidar_vec[0], msg.pose.pose.position.y + lidar_vec[1], 0),
                (q2[0], q2[1], q2[2], q2[3]),
                msg.header.stamp, 'laser_2', 'map')

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
        point_cloud.header.frame_id = "laser_2"
        self.point_cloud_pub.publish(point_cloud)

class Localization:
    def __init__(self, car_id, debug = False):
        self.error_pub = rospy.Publisher("/localization/error", Int16, queue_size=1)
        self.local_sub = rospy.Subscriber("/localization/odom/" + str(car_id), Odometry, self.localization_callback, queue_size=1)
        self.point_cloud_sub = rospy.Subscriber("/localization/point_cloud", PointCloud2, self.get_point_cloud, queue_size=1)
        self.positions = []
        self.tf = tf.TransformListener()
        self.debug = debug
        self.lane_id = 0
        # Steering and speed regulation
        self.car_x = None
        self.car_y = None
        self.p = 90
        self.k = -1
        self.obstacle_x = []
        self.obstacle_y = []
        self.old_time = None
        self.desired_speed = 600
        self.switched = False
        self.switched_at = None
        self.switched_mod = 0
        self.look_ahead_def = 100
        self.look_ahead_mod = 1
        self.look_ahead_curve_def = 100
        self.lanefree = [0]*2
        self.pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)
        self.pub_curr_speed = rospy.Publisher("/localization/curr_speed", Float64, queue_size=1, latch=False)
        self.pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
        self.sub_speed = rospy.Subscriber("/localization/desired_speed", Int16, self.get_desired_speed, queue_size=1)
        self.sub_look_ahead = rospy.Subscriber("/localization/look_ahead", Int16, self.get_look_ahead, queue_size=1)
        self.sub_look_ahead_curve = rospy.Subscriber("/localization/look_ahead_curve", Int16, self.get_look_ahead_curve, queue_size=1)
        # Shutdown control
        rospy.on_shutdown(self.shutdownhandler)

    def shutdownhandler(self):
        self.shutdown = 1
        self.pub_speed.publish(Int16(0))

    def get_point_cloud(self, msg):
        """Save current LIDAR PointCloud in Car coordinates"""
        if not self.car_x or not self.car_y:
            return

        self.lane_id = self.get_lane(msg)


    def get_desired_speed(self, msg):
        self.desired_speed = msg.data

    def get_look_ahead(self, msg):
        self.look_ahead_def = msg.data

    def speed(self):
        if self.switched:
            if self.switched_at > rospy.Time.now() - rospy.Duration(3):
                return max(self.desired_speed - 100, 400)
            else:
                self.look_ahead_mod = 1
                self.switched = False

        return self.desired_speed

    def get_look_ahead_curve(self, msg):
        self.look_ahead_curve_def = msg.data

    def lane_is_free(self, obstacles, lane_id):
        for p in obstacles:
            point = np.array(p)
            if abs(np.linalg.norm(point - self.closest_point(point, lane_id, 0))) < 16:
                self.obstacle_x.append(p[0])
                self.obstacle_y.append(p[1])
                self.lanefree[lane_id] = 5
                return False

        if self.lanefree[lane_id] > 0:
            self.lanefree[lane_id] -= 1
            return False

        return True

    def get_lane(self, point_cloud):
        """Transform and filter for obstacles in 1.5m distance"""
        obstacles = []
        for p in pc2.read_points(point_cloud):
            if p[0] > 0:
                """Ignore points behind the LIDAR"""
                continue
            ps = PointStamped(point_cloud.header, Point(p[0], p[1], p[2]))
            try:
                self.tf.waitForTransform('map', point_cloud.header.frame_id, point_cloud.header.stamp, rospy.Duration(0.15))
                new_ps = self.tf.transformPoint('map', ps)
            except tf.Exception as e:
                print('Can not transform points.')
                print("E: %s" % str(e))
                return self.lane_id

            """Our coordinates are in CM and switched because of the image plotting"""
            if abs(np.linalg.norm(np.array([self.car_x, self.car_y]) - np.array([new_ps.point.y * 100, new_ps.point.x * 100]))) < 150:
                obstacles.append([new_ps.point.y * 100, new_ps.point.x * 100])

        """Plot detected obstacles"""
        if self.debug:
            if len(obstacles) == 0:
                self.obstacles_x, self.obstacles_y = [], []
            else:
                self.obstacles_x, self.obstacles_y = np.array(obstacles).T

        """Check if obstacle is on either lane"""
        if self.lane_is_free(obstacles, self.lane_id):
            """STAY"""
            self.pub_speed.publish(Int16(self.speed()))
        elif self.lane_is_free(obstacles, (self.lane_id + 1) % 2):
            """SWITCH"""
            self.switched = True
            self.look_ahead_mod = 2
            self.switched_at = rospy.Time.now()
            self.lane_id = (self.lane_id + 1) % 2
            self.pub_speed.publish(Int16(self.speed()))
        else:
            """STOP"""
            self.pub_speed.publish(Int16(0))
            #rospy.signal_shutdown("STOP")

        return self.lane_id

    def calc_angle_rad(self, u_f, u_t):
        plane_normal = np.array([0, 0, 1])
        """Calculate angle"""
        angle = np.arccos(np.dot(u_f, u_t))
        """Calculate direction"""
        cross = np.cross(np.append(u_f, 0), np.append(u_t, 0))
        if np.dot(cross, plane_normal) <= 0:
            return min(angle, (np.pi / 2))
        else:
            return max(-1 * angle, (np.pi / -2))

    def look_ahead(self):
        return self.look_ahead_def


    def look_ahead_curve(self):
        return self.look_ahead_curve_def

    def calc_curr_speed(self, old_x, old_y, time):
        if not self.old_time:
            return 0
        self.pub_curr_speed.publish(Float64((abs(np.linalg.norm(np.array([old_x, old_y]) - np.array([self.car_x, self.car_y]))) / 100.0) / (time - self.old_time).to_sec()))

    def localization_callback(self, msg):
        """Odometry coordinates are in M ours are in CM and switched because of the image plotting"""
        car_x = msg.pose.pose.position.y * 100
        car_y = msg.pose.pose.position.x * 100

        """Move car ontop of front axel"""
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        """The QR-Code is approx 35cm from the front axle"""
        self.front_vec = np.array([np.sin(yaw), np.cos(yaw)]) * 35
        old_x, old_y = self.car_x, self.car_y
        self.car_x, self.car_y = self.front_vec + np.array([car_x, car_y])
        self.calc_curr_speed(old_x, old_y, msg.header.stamp)
        self.old_time = msg.header.stamp

        """Get lookahead based on car position"""
        if self.car_y <= (200) or self.car_y >= (404):
            look_ahead = self.look_ahead_curve() / self.look_ahead_mod
        else:
            look_ahead = self.look_ahead() / self.look_ahead_mod
        """Get next point on trajectory used to determine steering error"""
        self.distpoint = self.closest_point([self.car_x, self.car_y], self.lane_id, look_ahead)[-1]
        self.target_vec = self.distpoint - np.array([self.car_x, self.car_y])
        """Angle between car orientation and target vector"""
        u_f = self.front_vec / np.linalg.norm(self.front_vec)
        u_t = self.target_vec / np.linalg.norm(self.target_vec)
        self.angle_rad = self.calc_angle_rad(u_f, u_t)

        """This makes CaptainSteer for Localization obsolete"""
        self.steering_angle = (np.degrees(self.angle_rad) / self.k) + self.p

        """Rotate car vec by steering angle"""
        self.steering_vec = np.array([
            np.cos(np.radians(self.steering_angle - 90)) * self.front_vec[0] - np.sin(np.radians(self.steering_angle - 90)) * self.front_vec[1],
            np.sin(np.radians(self.steering_angle - 90)) * self.front_vec[0] + np.cos(np.radians(self.steering_angle - 90)) * self.front_vec[1]
            ])

        if self.debug:
            self.plot()

        self.pub_steering.publish(UInt8(self.steering_angle))

    def closest_point_on_circle(self, p, c, r, d):
        v = p - c
        closepoint = c + ((v * r) / np.linalg.norm(v))

        if d > 0:
            top = False
            if c[1] == 200:
                top = True
            dist = 0
            turnpoint = closepoint
            U = (2 * np.pi * r)
            if d > (U/2):
                temp = d % (U/2)
                dist = d - temp
                d = temp
            distangle = (360 * (d / U))
            if top:
                ca = np.array([1,0])
                pangle = np.degrees(np.arccos(np.dot(v,ca) / (np.linalg.norm(v) * np.linalg.norm(ca))))
                tangle = 360-(pangle + distangle) # reverse target angle
                if tangle < 180:
                    dist += (180 - tangle) * (U/360)
            else: #bot
                ca = np.array([-1,0])
                pangle = np.degrees(np.arccos(np.dot(v,ca) / (np.linalg.norm(v) * np.linalg.norm(ca))))
                tangle = 180-(pangle + distangle) # reverse target angle
                if tangle < 0:
                    dist += abs(tangle) * (U/360)
            cos = r * np.cos(np.deg2rad(tangle))
            sin = r * np.sin(np.deg2rad(tangle))
            turnpoint = np.array([cos, sin]) + c
        return turnpoint, dist, closepoint

    def closest_point(self, point, laneID, distance):
        """Returns closest point on trajectory."""
        # Top or bottom center of circle
        x, y = point[0], point[1]
        if laneID == 0: #INNERLANE
            laned = 16
        elif laneID == 1: #OUTERLANE
            laned = 48

        if x == 215 and y == 200:
            return np.array([215, 76 + laned])
        if x == 215 and y == 404:
            return np.array([215, 524 + laned])

        dp = [0,0]
        cp = [point]
        # Top half circle:
        distance += 1
        first = True
        while (distance > 0 or first):
            first = False
            if y <= 200:
                dp,distance,ncp = self.closest_point_on_circle(np.array([x, y]),
                        np.array([215, 200]), 121 + laned, distance)
                if distance > 0:
                    dp[0],dp[1] = 94 - laned, 201
            # Bottom half circle:
            elif y >= 404:
                dp,distance,ncp = self.closest_point_on_circle(np.array([x, y]), np.array([215, 404]),
                        121 + laned, distance)
                if distance > 0:
                    dp[0],dp[1] = 336 + laned, 403
            # Left line
            elif x <= 215:
                ncp = np.array([94 - laned, y])
                if y + distance > 404:
                    distance -= (404 - y)
                    dp = np.array([94 - laned, 404])
                else:
                    dp = np.array([94 - laned, y + distance])
                    distance = 0
            # Right line
            else:
                ncp = np.array([336 + laned, y])
                if y - distance < 200:
                    distance -= (y - 200)
                    dp = np.array([336 + laned, 200])
                else:
                    dp = np.array([336 + laned, y - distance])
                    distance = 0
            x,y = dp[0],dp[1]
            cp.append(ncp)
        cp.append(dp)
        return cp

    def plot(self):
        img = imread('texinput/pictures/map.png')
        plt.ion()
        plt.clf()
        plt.imshow(img)
        plt.plot(self.car_x, self.car_y, 'gx')
        plt.plot(self.obstacles_x, self.obstacles_y, 'y.')
        plt.plot(self.distpoint[0], self.distpoint[1], 'bx', label='angle to car %5.3f -> steering angle %5.3f' % (np.degrees(self.angle_rad), self.steering_angle))
        if self.obstacle_x:
            plt.plot(self.obstacle_x, self.obstacle_y, 'rx')
        plt.axes().arrow(self.car_x, self.car_y, self.front_vec[0], self.front_vec[1], head_width=4, head_length=6, color='w')
        plt.axes().arrow(self.car_x, self.car_y, self.target_vec[0], self.target_vec[1], head_width=4, head_length=6, color='r')
        plt.axes().arrow(self.car_x, self.car_y, self.steering_vec[0], self.steering_vec[1], head_width=4, head_length=6, color='b')
        plt.legend()
        plt.show()
        plt.pause(0.0001)

def main(args):
    rospy.init_node("oval_circuit")
    car_id  = 5

    tf = TFBroadcaster(car_id)
    lh = LazerHawk()
    localization = Localization(car_id, True)

    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
