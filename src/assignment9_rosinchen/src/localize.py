#!/usr/bin/env python

# --- imports ---

import rospy
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry



carID=7

import numpy as np

location = np.array([])
location_corr = np.array([])

centers = np.array([[196,215.5],[405,215.5]]) /100# upper, lower
lines   = np.array([[[196,95],[405,95]],[[196,336],[405,336]]])	/100
# lines = [left line, right line] with left/right line = [upper point, lower point]
radius  = 120.5 / 100

def nearest_point(given_point):
	x,y = given_point
	if x < centers[0,0]:	# upper circle
		vec_center_given = given_point - centers[0]
		vec_circle = vec_center_given *radius/np.linalg.norm(vec_center_given)
		return centers[0] + vec_circle

	elif x > centers[1,0]:	# lower circle
		vec_center_given = given_point - centers[1]
		vec_circle = vec_center_given *radius/np.linalg.norm(vec_center_given)
		return centers[1] + vec_circle

	elif y <=centers[0,1]: # left line
		return [given_point[0], lines[0,0,1]]

	elif y >centers[0,1]:	# right line
		return [given_point[0], lines[1,0,1]]

	else:
		print("ERROR in choice of track part!")
		stopTheScript()

def callback(data):
	global location 
	global location_corr
	x,y = data.pose.pose.position.x,data.pose.pose.position.y
	#print("x,y:",x,y)
	print(nearest_point((x,y))," ",x,y)
	location = np.append(location,([x,y]))
	location_corr = np.append(location_corr,(nearest_point((x,y))))
	#rospy.loginfo("x,y:",data)
	#print(location)
	np.save("location.npy", location)
	np.save("nearest_point.npy", location_corr)

rospy.init_node("localize", anonymous=True)

# create subscribers and publishers
sub_pos = rospy.Subscriber("/localization/odom/"+str(carID), Odometry, callback, queue_size=1)
# pub_mps = rospy.Publisher("mps", Float32, queue_size=1)

#Main

if rospy.is_shutdown():
	print("FUCK OFF")
	

#while not rospy.is_shutdown():
#	pass




# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
#m/s(auto) = (0.43182798*auto-34.99777655 )*0.0064381978570000001






