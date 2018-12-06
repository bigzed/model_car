#!/usr/bin/env python
import sys

#import matplotlib.pyplot as plt
import numpy as np
from collections import namedtuple

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, UInt8, UInt16

from time import localtime, strftime

inlier_dist = 0.01
plotting = False
sample_count = 50

# Driving
steering_angle = 0
speed_value = 250 #speed value
speed = speed_value # initial direction is backward

pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)

def scan_callback(msg):
	angles = []
	mask  = np.isfinite(msg.ranges)
	for i in range(len(msg.ranges)):
		angles.append(msg.angle_min + i*(msg.angle_increment))

	X,Y = np.array(poltocart(angles,msg.ranges))
	X = np.array(X[mask])
	Y = np.array(Y[mask])
	points = np.column_stack((X, Y))

	slope, intercept , slope2, intercept2 = find_perpendicularLines(points)
	print (slope, intercept , slope2, intercept2)
	#points = np.column_stack((X,Y))

        # A: Calculate distance to wall
        print("Distance: %s" % calculate_distance(np.array([0, 0]), slope, intercept))
        print("Distance: %s" % calculate_distance(np.array([0, 0]), slope2, intercept2))

        # B: Drive and thake 3 measurments
        for i in range(3):
            drive()

        drive(-3)


        return
	x = np.linspace(-1, 0.5, 2)
	plt.axis('equal')
	plt.ylim(-3,3)
	plt.plot(X,Y,'ro')
	plt.plot(x,(slope*x + intercept), color='blue')
	plt.plot(x,(slope2*x + intercept2), color='red')
	plt.draw()
	plt.show()

def drive(multiplier = 1):
    print("Drive")
    print(speed * multiplier)
    print(steering_angle)
    pub_steering.publish(UInt8(steering_angle))
    rospy.sleep(.5)
    pub_speed.publish(Int16(speed * multiplier))
    rospy.sleep(10)
    pub_speed.publish(0)


def calculate_distance(points, slope, intercept):
	""" return the distance for each point to the parametrised line """
	pos = np.array((0, intercept))  # origin
	dir = np.array((1, slope))  # line gradient
	# element-wise cross product of each points origin offset with the gradient
	c = np.cross(dir, pos - points, axisb=-1)

	return np.abs(c) / np.linalg.norm(dir)

def find_perpendicularLines(points):

	""" find the best params to describe a detected line using the RANSAC algorithm """
	best_count = 0
	best_count2 = 0
	best_params = (0,0)
	best_params2 = (0,0)
	xs, ys = points.T

	# randomly sample points to define a line and remember the one with the most inliers
	for _ in xrange(sample_count):
		if (len(xs)==0):
			print("warn: The wall couldn't be found!")
			continue
		ind = np.random.randint(0, len(xs), 2)
		x_a, x_b = xs[ind]
		if x_a == x_b:
			continue  # avoid division by 0

		y_a, y_b = ys[ind].astype(np.float64)

		slope = (y_b - y_a) / (x_b - x_a)
		intercept = y_a - x_a * slope

		inlier = get_inliers(points, slope, intercept)
		inl_count = np.sum(inlier)
		if inl_count > best_count:
			best_count = inl_count
			best_params = (slope, intercept)

	# the set of points within inlier distance of the best line we found
	#inlier_points = points[np.where(get_inliers(points, *best_params))]

	# perform a linear regression on the selected inlier points
	# slope, intercept, _, _, _ = stats.linregress(*inlier_points.T)


	slope, intercept = best_params

	## find the 2. line :
	slope2 = -( 1.0 / slope)

	for point in points:

		Px = point[0]
		Py = point[1]

		intercept2 = Py - (slope2*Px)

		inlier2 = get_inliers(points, slope2, intercept2)
		inl_count2 = np.sum(inlier2)
		if inl_count2 > best_count2:
			best_count2 = inl_count
			best_params2 = (slope2, intercept2)

	return slope, intercept , best_params2[0], best_params2[1]


def get_inliers(points, slope, intercept):
	""" return a numpy boolean array for each point (True if within 'inlier_dist' of the line, else False). """
	return get_distance(points, slope, intercept) <= inlier_dist


def get_distance(points, slope, intercept):
	""" return the distance for each point to the parametrised line """
	pos = np.array((0, intercept))  # origin
	dir = np.array((1, slope))  # line gradient
	# element-wise cross product of each points origin offset with the gradient
	c = np.cross(dir, pos - points, axisb=-1)
	return np.abs(c) / np.linalg.norm(dir)



def poltocart(angles,diss):
	X = []
	Y = []

	for i in range(len(angles)):
		X.append( diss[i] * np.cos(angles[i]))
		Y.append(diss[i] * np.sin(angles[i]))

	return X,Y

def main(args):
	global plotting
        global steering_angle
	rospy.init_node("angle_calibration")

        if len(args) > 1:
            try:
                    steering_angle = int(args[1])
                    rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
                    #rospy.Subscriber("/steering_angle", UInt16, steering_feedback_callback, queue_size=1)  # small queue for only reading recent data
            except rospy.ROSInterruptException:
                            pass

	if plotting:
		plt.show()  # block until plots are closed
	else:
		rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
