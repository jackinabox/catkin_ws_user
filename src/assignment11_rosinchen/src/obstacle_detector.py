#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np


class Obstacle_detector:

	def __init__(self, distanceToObstacleTreshold):
		self.distanceToObstacleTreshold = distanceToObstacleTreshold
		self.distanceToTrackTreshold = 0.15

	def awesome(self, lidar, position, model):
		obstacles = self.rotate(lidar, position)
		return self.process_obstacles(obstacles, model, position)

	def rotate(self, lidar, position):
		# global T
		x, y, w, z = position.pose.pose.position.x, position.pose.pose.position.y, position.pose.pose.orientation.w, position.pose.pose.orientation.z
		current_position = np.array([x, y])
		orientation_angle = 2 * np.arccos(w) * np.sign(z)
		T = np.array([[np.cos(orientation_angle), -np.sin(orientation_angle), 0, x],
					  [np.sin(orientation_angle), np.cos(orientation_angle), 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])

		# global T
		daten = lidar.ranges
		datenauto = np.zeros((360, 2))
		for inx, i in enumerate(daten):
			xauto = i * np.cos(inx)
			yauto = i * np.sin(inx)
			auto_vector = np.array([xauto, yauto, 0, 1]).reshape(4, 1)
			world_vector = np.dot(T, auto_vector)
			# print(world_vector[:2,0].shape)
			# datenauto = np.append(datenauto,world_vector[:2,0].reshape(2,1),axis=1)
			datenauto[inx, :] = world_vector[:2, 0].reshape(2, 1)
		print(datenauto[:, 1])
		return datenauto

	def process_obstacles(self, obstacles, model, position):
		x, y = position.pose.pose.position.x, position.pose.pose.position.y
		# car_lane = model.current_lane
		nearestPoints_obs = [model.nearest_point(obs) for obs in obstacles]
		nearestPoint_car = model.nearest_point([x, y])
		distanceToTrack = [np.linalg.norm(nearestPoints_obs[i], obstacles[i]) for i in range(len(nearestPoints_obs))]
		distanceToTrack[distanceToTrack > distanceToTrackTreshold] = np.nan
		distanceToCar = [np.linalg.norm(nearestPoints_obs[i], nearestPoint_car) for i in range(len(nearestPoints_obs))]
		nearestObstacle = np.minarg(distanceToCar)
		return distanceToCar[nearestObstacle] < distanceToObstacleTreshold
