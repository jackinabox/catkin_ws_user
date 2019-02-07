#!/usr/bin/env python

from nav_msgs.msg import Odometry
import roslib
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

import matplotlib.pyplot as plt

carID = 7
T = 0

# x,y=2.05,0.75
# current_position = np.array([x, y])
# orientation_angle = 0
# T = np.array([[np.cos(orientation_angle),-np.sin(orientation_angle),0,x],[np.sin(orientation_angle),np.cos(orientation_angle),0,y],[0,0,1,0],[0,0,0,1]])


# Model - values in cm
center_upper = [196, 215.5]
center_lower = [405, 215.5]
lines_inner = [[[196, 79], [405, 79]], [[196, 352], [405, 352]]]
lines_outer = [[[196, 47], [405, 47]], [[196, 384], [405, 384]]]
# lines = [left line, right line] with left line = [upper point, lower point]
radius_inner = 136.5
radius_outer = 168.5
twoLines = np.array((lines_inner, lines_outer)) / 100.  # convert to meter
twoRadii = np.array((radius_inner, radius_outer)) / 100.
twoCenters = np.array((center_upper, center_lower)) / 100.
current_lane = 0
logging = False
Lanes = {0: "inner lane", 1: "outer lane"}

plt.figure()
print("start obsticle_detection.py")


def callback_position(data):
	global T
	global current_position
	global x, y

	x, y, w, z = data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.w, data.pose.pose.orientation.z

	# print("current pos",x,y)
	current_position = np.array([x, y])
	orientation_angle = 2 * np.arccos(w) * np.sign(z)
	T = np.array([[np.cos(orientation_angle), -np.sin(orientation_angle), 0, x],
				  [np.sin(orientation_angle), np.cos(orientation_angle), 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])


def callback_scan(data):
	global T
	global current_position
	global x, y
	daten = data.ranges
	inc = data.angle_increment  # 0.0174532923847
	# print(inc)
	# print(daten)
	datenauto = np.zeros((360, 2)) + np.nan

	# print(len(daten))
	for inx, i in enumerate(daten):

		if inx > 60 and inx < 300:
			# print('1')
			continue
		elif i > 0.7:
			# print('2')
			continue
		# print('0')
		inxr = inx * np.pi / 180  # -np.pi + (inc*inx) #*np.pi/180
		xauto = i * np.cos(inxr)
		yauto = i * np.sin(inxr)
		auto_vector = np.array([xauto, yauto, 0, 1]).reshape(4, 1)
		world_vector = np.dot(T, auto_vector)
		# print(world_vector[:2,0].shape)
		datenauto[inx, :] = world_vector[:2, 0].reshape(1, 2)

	# print(datenauto[:,:])

	# car_lane = model.current_lane
	nearestPoints_obs = np.array([nearest_point(obs)[1] for obs in datenauto])
	# print("NearObs:",nearestPoints_obs[0])
	nearestPoint_car = nearest_point([x, y])[1]
	# print("NearCar:",nearestPoint_car)
	# print("nearestPoints_obs: ", nearestPoints_obs)
	distanceToTrack = np.array(
		[np.linalg.norm(nearestPoints_obs[i] - datenauto[i]) for i in range(len(nearestPoints_obs))])
	# print(len(distanceToTrack),len(nearestPoints_obs))
	# distanceToTrack[distanceToTrack > 0.5] = np.nan
	# nearestPoints_obs[distanceToTrack > 0.5] = np.nan

	distanceToCar = np.array(
		[np.linalg.norm(nearestPoints_obs[i] - nearestPoint_car) for i in range(len(nearestPoints_obs))])
	distanceToCar[distanceToCar < 0.2] = np.nan
	nearestPoints_obs[distanceToCar < 0.2] = np.nan
	nearestObstacle = np.nanargmin(distanceToCar)
	print(distanceToCar[nearestObstacle])
	datenauto[np.isinf(datenauto)] = np.nan
	# datenauto[np.isnan(datenauto)] = 43
	plt.cla()
	# print(distanceToCar[nearestObstacle])
	plt.scatter(nearestPoint_car[0], nearestPoint_car[1], marker="H", color="k")
	plt.scatter(datenauto[:, 0], datenauto[:, 1], color="r")
	plt.scatter(nearestPoints_obs[:, 0], nearestPoints_obs[:, 1], color="g")
	plt.plot(current_position[0], current_position[1], marker="H")
	plt.show(block=False)


def nearest_point(given_point):
	if np.isnan(given_point[0]):
		return np.nan, np.array([0, 0]) + np.nan
	curr_lane = current_lane
	lines = twoLines[curr_lane]
	radius = twoRadii[curr_lane]
	centers = twoCenters
	x, y = given_point

	if x < centers[0, 0]:  # upper semicircle
		vec_center_given = given_point - centers[0]
		vec_circle = vec_center_given * radius / np.linalg.norm(vec_center_given)
		return 0, np.array(centers[0] + vec_circle)

	elif x > centers[1, 0]:  # lower semicircle
		vec_center_given = given_point - centers[1]
		vec_circle = vec_center_given * radius / np.linalg.norm(vec_center_given)
		return 1, np.array(centers[1] + vec_circle)

	elif y <= centers[0, 1]:  # left line
		return 2, np.array([x, lines[0, 0, 1]])

	elif y > centers[0, 1]:  # right line
		return 3, np.array([x, lines[1, 0, 1]])

	else:
		print("ERROR in choice of track part!")


rospy.init_node("obstacle_detection", anonymous=True)

sub_pos = rospy.Subscriber("/localization/odom/" + str(carID), Odometry, callback_position, queue_size=1)
sub_scan = rospy.Subscriber("/scan", LaserScan, callback_scan, queue_size=1)

plt.show(block=True)
# rospy.spin()
