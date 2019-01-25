#!/usr/bin/env python
import sys
import rospy
import matplotlib.pyplot as plt
import numpy as np
import roslib
import sys
import cv2
import time

#from sklearn import linear_model
from scipy.misc import imread
from cv_bridge import CvBridge, CvBridgeError
from collections import namedtuple
from sensor_msgs.msg import Image
from std_msgs.msg import Int16, UInt8, UInt16, Float64
from collections import deque
from nav_msgs.msg import Odometry
from numpy.linalg import norm

from os.path import expanduser

class ClosestPointWithPrediction:
    def __init__(self):
        home = expanduser("~")
        self.img_path = '/git/model_car/texinput/pictures/map.png'
        self.img_path = home + self.img_path

        # input values here
        self.carPoint = np.array([300, 200])
        self.LANEID = 1
        self.DISTANCE = 50
        
    def getClosestPoint(self, point, laneID,  distance):
        x, y = point[0], point[1]

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
        #print(distanceangle)

        ## c2 = np.cos(distanceangle)
        ## s2 = np.sin(distanceangle)
        ## distpoint[0] = distpoint[0]*c + distpoint[1]*(-s)
        ## distpoint[1] = distpoint[0]*s + distpoint[1]*(c)

        circleOrigin = c
        car = p
        radius = r

        """ Parameterdarstellung der Koordinaten: https://de.wikipedia.org/wiki/Kreis#Parameterdarstellung """
        # if car is on top right corner
        if car[0] > 215: # and car[0] < distpoint[0]:
            distpoint[0] = c[0] + (r * np.cos(distanceangle * np.pi/180.0))
            distpoint[1] = c[1] - (r * np.sin(distanceangle * np.pi/180.0))

        # TODO if car is on top left corner , reduntant ???
        # TODO if car is on bottom right corner
        # TODO if car is on bottom left corner
        return distpoint

    def plot(self):
        img = imread(self.img_path)
        plt.imshow(img)

        # plot car YELLOW
        plt.plot(self.carPoint[0], self.carPoint[1], 'y+')
        print('Car point:', self.carPoint)

        #plot point ahead MAGENTA
        closestPoint = self.getClosestPoint(self.carPoint, int(self.LANEID), int(self.DISTANCE))
        plt.plot(int(closestPoint[0]), int(closestPoint[1]), 'm+')
        print('Prediction:', closestPoint)
        
        plt.show(block=True)

def main(args):
    cpd = ClosestPointWithPrediction()
    cpd.plot()

if __name__ == '__main__':
    main(sys.argv)
