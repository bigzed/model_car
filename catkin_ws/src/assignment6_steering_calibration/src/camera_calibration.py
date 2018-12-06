#!/usr/bin/env python
import sys

import matplotlib.pyplot as plt
import numpy as np
from collections import namedtuple
from scipy import optimize

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, UInt8, UInt16

from time import localtime, strftime,sleep


pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=100, latch=True)
pub_steering = rospy.Publisher("/steering", UInt8, queue_size=100, latch=True)

global car_cor_x
global car_cor_y
car_cor_y = []
car_cor_x = []

inlier_dist = 0.05
sample_count = 50
plt.ion()
plt.show()

def scan_callback(msg):
	global lidar_calc
	angles = []
	mask = np.array(msg.ranges<1)

	ranges = np.array(msg.ranges)
	mask = np.array(ranges<1.5)
	#mask  = np.isfinite(msg.ranges)

	for i in range(len(ranges)):
		angles.append(msg.angle_min + i*(msg.angle_increment))

	X,Y = np.array(poltocart(angles,ranges))
	X = np.array(X[mask])
	Y = np.array(Y[mask])
	points = np.column_stack((X, Y))

	slope, intercept , slope2, intercept2 = find_perpendicularLines(points)
	#print (slope, intercept , slope2, intercept2)

	x = np.linspace(-5, 5, 2)
	plt.clf()
	plt.axis('equal')
	plt.ylim(-3,3)
	plt.plot(X,Y,'ro')
	plt.plot(x,(slope*x + intercept), color='blue')
	plt.plot(x,(slope2*x + intercept2), color='red')
	plt.pause(0.001)
	if lidar_calc:
		cor = calculate_global_cor(slope, intercept , slope2, intercept2)
		car_cor_x.append(cor[0])
		car_cor_y.append(cor[1])
		lidar_calc = False

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
	inlier_points = points[np.where(get_inliers(points, *best_params))]

	slope, intercept = best_params

	## find the 2. line :
	slope2 = -( 1.0 / slope)

	for point in inlier_points:

		Px = point[0]
		Py = point[1]

		intercept2 = Py - (slope2*Px)

		inlier2 = get_inliers(points, slope2, intercept2)
		inl_count2 = np.sum(inlier2)
		if inl_count2 > best_count2:
			best_count2 = inl_count
			best_params2 = (slope2, intercept2)

		slope2, intercept2 = best_params2
	return slope, intercept , slope2, intercept2


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


def drive(multiplier , steering_angle ,speed ):

    print("Drive")
    pub_steering.publish(UInt8(steering_angle))
    rospy.sleep(.5)
    pub_speed.publish(Int16(speed))
    rospy.sleep(1 * multiplier)
    pub_speed.publish(0)

def calculate_global_cor(slope1, intercept1,slope2, intercept2):

	x_global = get_distance([0,0],slope1, intercept1)
	y_global = get_distance([0,0],slope2, intercept2)

	return (x_global, y_global)

#############################
def calc_R(x,y, xc, yc):
    """ calculate the distance of each 2D points from the center (xc, yc) """
    return np.sqrt((x-xc)**2 + (y-yc)**2)

def f(c, x, y):
    """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(x, y, *c)
    return Ri - Ri.mean()

def leastsq_circle(x,y):
    # coordinates of the barycenter
    x_m = np.mean(x)
    y_m = np.mean(y)
    center_estimate = x_m, y_m
    center, ier = optimize.leastsq(f, center_estimate, args=(x,y))
    xc, yc = center
    Ri       = calc_R(x, y, *center)
    R        = Ri.mean()
    residu   = np.sum((Ri - R)**2)
    return  R

##################################

def calc_gamma(L,R):
	return np.arctan(L/R)


def do_calc(angles):
	global car_cor_x
	global car_cor_y
	global lidar_calc

	for angle in angles:
		car_cor_x = []
		car_cor_y = []

		for i in range(3):
		# Lidar data calculation
			lidar_calc = True
			rospy.sleep(3)

			lidar_calc = False
			drive(1,angle,-200)


		r = leastsq_circle(car_cor_x,car_cor_y)
		print("R is %s"%(r))
		gamma = calc_gamma(0.26,r)

		print("Gamma is %s"%(gamma))

		drive(3,angle,200)
		x = raw_input("Press anykey after reputting the car in the correct place ")



def main(args):

	global lidar_calc
	lidar_calc = False
	rospy.init_node("angle_calibration")

	try:
		rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
		#rospy.Subscriber("/steering_angle", UInt16, steering_feedback_callback, queue_size=1)  # small queue for only reading recent data
	except rospy.ROSInterruptException:
		pass

	do_calc([0, 30, 60, 90, 120, 150, 180])

	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
