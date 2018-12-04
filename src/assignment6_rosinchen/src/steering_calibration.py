#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
#from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt

deg_step = 0.01745
ransac_threshold = 0.15
numberOfIterations = 80
#samplerate = 0.05
eps = np.finfo(float).eps

dist_threshold = 2


def get_m_b(vec0, vec1):
	m = (vec1[1] - vec0[1]) / (vec1[0] - vec0[0] + eps)
	b = lambda x: int((x - vec0[0]) * m + vec0[1])
	return m, b(0), b

def getPointsFromLine(line):
	m, b = line
	edge = [None, None]
	edge[0] = (0, int(b))
	edge[1] = (image_shape[0], int(m * (image_shape[0]) + b))
	return edge


def ransac(points):
	results = np.zeros((numberOfIterations, 3))
	inliersOfIterations = {}
	for iter in range(numberOfIterations):
		choice = points[np.random.choice(range(points.shape[0]), 2, replace=False)]
		# rospy.loginfo("choice:" + str(choice))
		m, b, b_func = get_m_b(choice[0], choice[1])

		inliers = np.zeros(points.shape[0])
		for index, point in enumerate(points):
			if abs(point[1] - b_func(point[0])) <= ransac_threshold:
				inliers[index] = 1
		inliersOfIterations[iter] = inliers
		results[iter, :] = [m, b, sum(inliers)]
	# rospy.loginfo("   m = %f; b = %f" % (m, b))
	#rospy.loginfo(str(results[:, 2]))
	indBest = np.argmax(results[:, 2])
	best_result = results[indBest, :]
	rospy.loginfo("best results: " + str(best_result))
	m, b, _ = best_result
	return (m, b), inliersOfIterations[indBest]

def get_point_coords_from_scan(ranges):
	coords = np.zeros((len(ranges), 2))
	for ind, dist in enumerate(ranges):
		angle = -np.pi + (ind * deg_step)
		if dist <= dist_threshold:
			x = dist * np.cos(angle)
			y = dist * np.sin(angle)
		else:
			x, y = np.inf, np.inf
		coords[ind] = np.array([x, y])
	coords = coords[np.isfinite(coords)]
	return coords.reshape((len(coords)//2, 2))

def plot_points(points):
	rospy.loginfo("dim points for plot:" + str(points.shape))
	x = points[:, 0]
	y = points[:, 1]
	#colors = np.random.rand(N)
	#area = (30 * np.random.rand(N)) ** 2  # 0 to 15 point radii

	plt.scatter(x, y)
	plt.show(block=False)

def plotter(m,n,daten):
	f = lambda x: m*x+n
	x = np.linspace(-0.79, -0.7)
	y = f(x)
	plt.scatter(daten[:, 0], daten[:, 1])
	plt.scatter(x, y)
	plt.show(block=False)

def callback(raw_msg):
	rospy.loginfo("start callback")
	scan_points = get_point_coords_from_scan(raw_msg.ranges)
	rospy.loginfo(scan_points)
	line_one, inliers_one = ransac(scan_points)
	points_subset = scan_points[inliers_one == 0]
	line_two, inliers_two = ransac(points_subset)
	#plot_points(points)
	plotter(line_one[0], line_one[1], points_subset)
	#plotter(line_two[0], line_two[1], scan_points)
	#plot_points(scan_points[inliers_one == 0])
	#plot_points(points_subset[inliers_two == 1])
	#plotter(points[inliers_one == 0][0])
	#rospy.loginfo("head: "+str(raw_msg.ranges[:10]))
	#rospy.loginfo("tail: " + str(raw_msg.ranges[-10:]))
	#rospy.loginfo("min dist: " + str(np.min(raw_msg.ranges)))
	#rospy.loginfo("incr: " + str(raw_msg.angle_increment))
	#rospy.loginfo("points: " + str(points))
	#pub_scan.publish("I heard scan: "+str(scan))

rospy.init_node("node_wall_scan")
rospy.loginfo("start publish and subscribe")
#plot_lines()
pub_scan = rospy.Publisher("/wall_scan", Float32, queue_size=1)
rospy.Subscriber("/scan", LaserScan, callback)
rospy.loginfo("done.")

rospy.spin()