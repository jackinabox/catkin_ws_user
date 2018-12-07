#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
# from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
import numpy as np

# import matplotlib
# matplotlib.use('TkAgg')
# from matplotlib import pyplot as plt

deg_step = 0.01745
ransac_threshold = 0.15
numberOfIterations = 80
# samplerate = 0.05
eps = np.finfo(float).eps

dist_threshold = 1

dist_threshold = 2
scanNow=False
punkte = []#np.array([[0,0],[0,0],[0,0]])


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
	# rospy.loginfo(str(results[:, 2]))
	indBest = np.argmax(results[:, 2])
	best_result = results[indBest, :]
	#rospy.loginfo("best results: " + str(best_result))
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
	return coords.reshape((len(coords) // 2, 2))


def plot_points(points):
	#rospy.loginfo("dim points for plot:" + str(points.shape))
	x = points[:, 0]
	y = points[:, 1]
	# colors = np.random.rand(N)
	# area = (30 * np.random.rand(N)) ** 2  # 0 to 15 point radii

	plt.scatter(x, y)
	plt.show(block=False)


def plotter(m, n, daten):
	f = lambda x: m * x + n
	x = np.linspace(-0.79, -0.7)
	y = f(x)
	plt.scatter(daten[:, 0], daten[:, 1])
	plt.scatter(x, y)
	plt.show(block=False)


def distance_to_wall(ranges, dx):
	# x = np.linspace(0, 359./360 * 2 * np.pi, 360)
	x = np.arange(-100, 100, 1)
	fit = np.polyfit(x, ranges, 4)

	#rospy.loginfo(" 1. fit  :" + str(fit))
	der = np.polyder(fit)
	#rospy.loginfo(" 2. deriv:" + str(der))
	root = np.roots(der)
	#rospy.loginfo(" 3. root :" + str(root))
	indices = np.array(np.round(root, 0), dtype=int)
	#rospy.loginfo(" 4. indices  :" + str(indices))
	wall_deg = ranges[indices]
	#rospy.loginfo(" 4. values  :" + str(wall_deg))
	choice = [0, 1]  # two the three values represent the walls
	dist = wall_deg[choice]
	#rospy.loginfo(" 5. dist  :" + str(dist))

	#rospy.loginfo(" 1. fit  :"+str(fit))
	der=np.polyder(fit)
	#rospy.loginfo(" 2. deriv:"+str(der))
	root=np.roots(der)
	#rospy.loginfo(" 3. root :"+str(root))

	# check, whether max or min via sign of second derivative
	#der2=np.poly1d(der)
	#sign=der2(root)
	#rospy.loginfo(" 3b. sign:"+str(sign))
	#choice=sign>0
	choice=[0,1]

	indices = np.array(np.round(root,0), dtype=int)
	#rospy.loginfo(" 4. indices  :"+str(indices))
	wall_deg=ranges[indices]
	#rospy.loginfo(" 4. values  :"+str(wall_deg))
	dist=wall_deg[choice]
	#rospy.loginfo(" 5. dist  :"+str(dist))


	return dist


def interpolate(arr):
	for ind, val in enumerate(arr):
		if not np.isfinite(val):
			arr[ind] = arr[ind - 1]
	return arr




counter = 0
times = 0

achsenabstand = 0.2725
abstandhinten = 0.2075


def lenkwinkel(koordinaten):
	x = koordinaten[:, 0]
	y = koordinaten[:, 1]

	A = np.concatenate((np.ones((3,1)), -x.reshape(3,1), -y.reshape(3,1)),axis=1)

	b = -np.array([(x[0] ** 2 + y[0] ** 2), (x[1] ** 2 + y[1] ** 2), (x[2] ** 2 + y[2] ** 2)])
	#rospy.loginfo(str(x) + str(x[1])+str(x[2]))
	#rospy.loginfo(str(A) + str(b))
	loes = np.linalg.solve(A, b.T)

	R = np.sqrt(0.25 * (loes[1] ** 2 + loes[2] ** 2) - loes[0])
	winkel = np.arctan(achsenabstand / np.sqrt(R ** 2 - abstandhinten ** 2))
	winkeldeg = winkel *360/(2*np.pi)
	return loes, R,winkeldeg


def callback(raw_msg):

	global counter

	rospy.loginfo("start callback")

	#rospy.loginfo("start callback")


	ranges = np.array(raw_msg.ranges, dtype=float)

	# rospy.loginfo(raw_msg.ranges)
	indices = np.arange(360)
	points = np.concatenate((ranges[260:], ranges[:100]))
	indices = np.concatenate((indices[260:], indices[:100]))
	# rospy.loginfo("points: " + str(points))
	points = interpolate(points)
	# rospy.loginfo("points inter: " + str(points))

	# RANSAC:

	# line_one, inliers_one = ransac(scan_points)
	# points_subset = scan_points[inliers_one == 0]
	# line_two, inliers_two = ransac(points_subset)
	# points = raw_msg.ranges[np.isfinite(raw_msg.ranges)]
	test = np.array([(1, 0), (0, 1), (-1, 0)])

	rospy.loginfo("Loes: " + str(lenkwinkel(test)))

	rospy.loginfo("counter: "+str(counter))
	if counter <= 2:
		dist = distance_to_wall(points, deg_step)
		rospy.loginfo("dist:"+str(dist)+str(dist[0])+str(dist[1]))
		global punkte 
		punkte.append([dist[0],dist[1]])
		rospy.loginfo("punkte:"+str(punkte))
		counter += 1
		rospy.loginfo("Loes: Weiterfahren!" )
		rospy.sleep(10)

	if counter>2:
		counter = 0
		rospy.loginfo(str(punkte))
		winkel = lenkwinkel(np.array(punkte))
		rospy.loginfo(str(winkel)+"Loes: Neuen Lenkwinkel einstellen!")

		#winkel = lenkwinkel(punkte)
		#winkel aendern
		rospy.exit()
		#global punkte
		#punkte = []
		rospy.sleep(2)

	# root = np.array( np.round(roots(points, deg_step),0), dtype=int)
	#rospy.loginfo(" ------ distance :" + str(dist))


# rospy.loginfo("distance to wall  :"+str(points[root]))

# choice=[0,2] # from measurements we know, that the first and third element give distances to the wall
# dist=root[choice]
# rospy.loginfo("distance to wall  :"+str(points[root]))


# plot_points(points)
# plotter(line_one[0], line_one[1], scan_points)

# plot_points(scan_points[inliers_one == 0])
# plot_points(points_subset[inliers_two == 1])
# plotter(points[inliers_one == 0][0])
# rospy.loginfo("head: "+str(raw_msg.ranges[:10]))
# rospy.loginfo("tail: " + str(raw_msg.ranges[-10:]))
# rospy.loginfo("min dist: " + str(np.min(raw_msg.ranges)))
# rospy.loginfo("incr: " + str(raw_msg.angle_increment))
# rospy.loginfo("points: " + str(points))
# pub_scan.publish("I heard scan: "+str(scan))

	#line_one, inliers_one = ransac(scan_points)
	#points_subset = scan_points[inliers_one == 0]
	#line_two, inliers_two = ransac(points_subset)
	#points = raw_msg.ranges[np.isfinite(raw_msg.ranges)]
	dist=distance_to_wall(points,deg_step)

	#root = np.array( np.round(roots(points, deg_step),0), dtype=int)
	rospy.loginfo(" ------ distance :"+str(dist)+" -------")
	#rospy.loginfo("distance to wall  :"+str(points[root]))
	
	#choice=[0,2] # from measurements we know, that the first and third element give distances to the wall
	#dist=root[choice]
	#rospy.loginfo("distance to wall  :"+str(points[root]))
	


	#plot_points(points)
	#plotter(line_one[0], line_one[1], scan_points)

	#plot_points(scan_points[inliers_one == 0])
	#plot_points(points_subset[inliers_two == 1])
	#plotter(points[inliers_one == 0][0])
	#rospy.loginfo("head: "+str(raw_msg.ranges[:10]))
	#rospy.loginfo("tail: " + str(raw_msg.ranges[-10:]))
	#rospy.loginfo("min dist: " + str(np.min(raw_msg.ranges)))
	#rospy.loginfo("incr: " + str(raw_msg.angle_increment))
	#rospy.loginfo("points: " + str(points))
	#pub_scan.publish("I heard scan: "+str(scan))
	#rospy.sleep(1)


rospy.init_node("node_wall_scan")
rospy.loginfo("start publish and subscribe")
# plot_lines()
pub_scan = rospy.Publisher("/wall_scan", Float32, queue_size=1)
rospy.Subscriber("/scan", LaserScan, callback)
rospy.loginfo("hallo")
rospy.loginfo("done.")

if counter<3:
	rospy.spin()
