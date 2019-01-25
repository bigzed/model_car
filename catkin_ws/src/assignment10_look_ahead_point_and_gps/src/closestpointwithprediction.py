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
        self.img_path = '/git/model_car/texinput/pictures/map.png'
        self.img_path = expanduser("~") + self.img_path
        
	self.cpToCar = np.array([0,0])
        
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

    def dist_on_circle(self, carPoint, circlePoint, r, distance):
	# vector from car to circle origin
        vcarToCircle = carPoint - circlePoint
	print('vcarToCircle: ', vcarToCircle)

	# calculate closest point from car
        distpoint = circlePoint + (vcarToCircle * r / np.linalg.norm(vcarToCircle))
	self.cpToCar[:] = distpoint

	# calculate the circuit of the circle        
	circuit = 2 * np.pi * r
	# calculate the desired distance in degrees
        distanceangle = 360 * (distance / circuit) 
	print('distanceangle: ', distanceangle)

        """ Parameterdarstellung der Koordinaten: https://de.wikipedia.org/wiki/Kreis#Parameterdarstellung """
        # if car is on top right corner
        if carPoint[0] > 215 and carPoint[1] < 196:
		# TODO experimental
		# the car would drive to bottom right instead of top left so we go backwards from the circle
		if carPoint[1] < (196 - (196/2)) and carPoint[0] > (215 + (215/2)):
			newangle = 90 - (90 * (distance / circuit))
			print('newangle: ', newangle)
			distanceangle = newangle
        	distpoint[0] = circlePoint[0] + (r * np.cos(distanceangle * np.pi/180.0))
        	distpoint[1] = circlePoint[1] - (r * np.sin(distanceangle * np.pi/180.0))

        # if car is on top left corner
        if carPoint[0] < 215 and carPoint[1] < 196:
        	distpoint[0] = circlePoint[0] - (r * np.cos(distanceangle * np.pi/360.0))
        	distpoint[1] = circlePoint[1] - (r * np.sin(distanceangle * np.pi/360.0))        

	# if car is on bottom right corner
	if carPoint[0] > 215 and carPoint[1] > 415:
        	distpoint[0] = circlePoint[0] + (r * np.cos(distanceangle * (np.pi)/360.0))
        	distpoint[1] = circlePoint[1] + (r * np.sin(distanceangle * (np.pi)/360.0))		
	
	# if car is on bottom left corner
	if carPoint[0] < 215 and carPoint[1] > 415:
        	distpoint[0] = circlePoint[0] - (r * np.cos(distanceangle * (np.pi)/180.0))
        	distpoint[1] = circlePoint[1] + (r * np.sin(distanceangle * (np.pi)/180.0))

	return distpoint

    def plot(self, point, distance, laneID):
        img = imread(self.img_path)
        plt.imshow(img)

        self.carPoint = point
        self.LANEID = laneID
        self.DISTANCE = distance

        # plot car YELLOW
        plt.plot(self.carPoint[0], self.carPoint[1], 'y+')
        
        #plot point ahead MAGENTA
        closestPoint = self.getClosestPoint(self.carPoint, int(self.LANEID), int(self.DISTANCE))
        plt.plot(int(closestPoint[0]), int(closestPoint[1]), 'm+')

	# plot closest point to car GREEN
	plt.plot(self.cpToCar[0], self.cpToCar[1], 'g+')
	
	print('Car point:', self.carPoint)
	print('closestPointToCar:' , self.cpToCar)
        print('Prediction:', closestPoint)
        
        plt.show(block=True)

def main(args):
    # task 1.1
    p = np.array([300, 200])
    d = 50 
    l = 1
    cpd = ClosestPointWithPrediction()
    cpd.plot(p, d, l)
    
    # task 1.2
    p = np.array([100, 100])
    d = 20
    l = 2
    cpd2 = ClosestPointWithPrediction()
    cpd2.plot(p, d, l)

    # bottom right, outer lane, closest to turn from south to north
    p = np.array([250, 590])
    d = 70
    l = 2
    cpd3 = ClosestPointWithPrediction()
    cpd3.plot(p, d, l)

    # bottom right, closest to circle origin
    p = np.array([250, 450])
    d = 70
    l = 2
    cpd4 = ClosestPointWithPrediction()
    cpd4.plot(p, d, l)

    # bottom right, farest from circle origin
    p = np.array([400, 550])
    d = 70
    l = 2
    cpd5 = ClosestPointWithPrediction()
    cpd5.plot(p, d, l)

    # bottom right, outer lane, closest y to straight line y
    p = np.array([410, 440])
    d = 20
    l = 2
    cpd6 = ClosestPointWithPrediction()
    cpd6.plot(p, d, l)

    # bottom right, inner lane, closest y to straight line y
    p = np.array([370, 440])
    d = 20
    l = 2
    cpd7 = ClosestPointWithPrediction()
    cpd7.plot(p, d, l)

    # right lane, inner lane
    p = np.array([370, 340])
    d = 20
    l = 2
    cpd8 = ClosestPointWithPrediction()
    cpd8.plot(p, d, l)

    # top right
    p = np.array([410, 190])
    d = 20
    l = 2
    cpd9 = ClosestPointWithPrediction()
    cpd9.plot(p, d, l)  

    # top right
    p = np.array([410, 50])
    d = 20
    l = 2
    cpd10 = ClosestPointWithPrediction()
    cpd10.plot(p, d, l) 	

if __name__ == '__main__':
    main(sys.argv)
