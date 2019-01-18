#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

carID = 7
location = np.array([])
location_corr = np.array([])

# Model
centers = np.array([[196, 215.5], [405, 215.5]]) / 100.0  # upper, lower
lines = np.array([[[196, 95], [405, 95]], [[196, 336], [405, 336]]]) / 100.0
radius = 120.5 / 100


def nearest_point(given_point):
	x, y = given_point
	if x < centers[0, 0]:  # upper semicircle
		vec_center_given = given_point - centers[0]
		vec_circle = vec_center_given * radius/np.linalg.norm(vec_center_given)
		return np.array(centers[0] + vec_circle)

	elif x > centers[1, 0]:  # lower semicircle
		vec_center_given = given_point - centers[1]
		vec_circle = vec_center_given * radius/np.linalg.norm(vec_center_given)
		return np.array(centers[1] + vec_circle)

	elif y <= centers[0, 1]:  # left line
		return np.array([given_point[0], lines[0, 0, 1]])

	elif y > centers[0, 1]:  # right line
		return np.array([given_point[0], lines[1, 0, 1]])

	else:
		print("ERROR in choice of track part!")


def callback(data):
	global location 
	global location_corr
	x, y = data.pose.pose.position.x, data.pose.pose.position.y
	#print("x,y:",x,y)
	print(nearest_point((x, y)), " ", x, y)
	location = np.append(location, ([x, y]))
	location_corr = np.append(location_corr, (nearest_point((x, y))))
	#rospy.loginfo("x,y:",data)
	#print(location)
	np.save("location.npy", location)
	np.save("nearest_point.npy", location_corr)


rospy.init_node("localize", anonymous=True)

# create subscribers and publishers
sub_pos = rospy.Subscriber("/localization/odom/"+str(carID), Odometry, callback, queue_size=1)

rospy.spin()






