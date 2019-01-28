#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np


# import matplotlib.pyplot as plt


class ObstacleDetector:

	def __init__(self, distanceToObstacleTreshold, logging):
		self.distanceToObstacleTreshold = distanceToObstacleTreshold
		self.distanceToTrackTreshold = 0.15
		self.radius_threshold = 1.5
		self.logging = logging

	def detects_an_obstacle(self, lidar, position, model):
		obstacles = self.rotate(lidar, position)
		return self.process_obstacles(obstacles, model, position)

	def rotate(self, lidar, position):
		print("_____rotate() ....")
		# global T
		x, y, w, z = position.pose.pose.position.x, position.pose.pose.position.y, position.pose.pose.orientation.w, position.pose.pose.orientation.z
		# current_position = np.array([x, y])
		orientation_angle = 2 * np.arccos(w) * np.sign(z)
		T = np.array([[np.cos(orientation_angle), -np.sin(orientation_angle), 0, x],
					  [np.sin(orientation_angle), np.cos(orientation_angle), 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])

		# global T
		daten = lidar.ranges
		datenauto = np.zeros((360, 2))
		for inx, i in enumerate(daten):
			if i > self.radius_threshold:
				continue
			inxr = inx * 180 / np.pi
			xauto = i * np.cos(inxr)
			yauto = i * np.sin(inxr)
			auto_vector = np.array([xauto, yauto, 0, 1]).reshape(4, 1)
			if self.logging:
				print("rotate() -> auto_vector.shape: ", auto_vector.shape)
			world_vector = np.dot(T, auto_vector)
			if self.logging:
				print("rotate() -> world_vector.shape: ", world_vector.shape)
			datenauto[inx, :] = world_vector[:2, 0].reshape(1, 2)
			if self.logging:
				print("rotate() -> world_vector[:2, 0].reshape(1, 2): ", world_vector[:2, 0].reshape(1, 2))
			if self.logging:
				print("rotate() -> datenauto: ", datenauto.shape)
		# plt.scatter(world_vector[0],world_vector[1])
		# plt.show()
		print("....rotate()_______")
		datenauto[np.isinf(datenauto)] = 42
		datenauto[np.isnan(datenauto)] = 43

		# print("datenauto: ",datenauto)
		return datenauto

	def process_obstacles(self, obstacles, model, position):
		print('_____process_obstacles() .....')
		x, y = position.pose.pose.position.x, position.pose.pose.position.y
		# car_lane = model.current_lane
		nearestPoints_obs = [model.nearest_point(obs)[1] for obs in obstacles]
		nearestPoint_car = model.nearest_point([x, y])[1]
		# print("nearestPoints_obs: ", nearestPoints_obs)
		distanceToTrack = np.array(
			[np.linalg.norm(nearestPoints_obs[i] - obstacles[i]) for i in range(len(nearestPoints_obs))])
		distanceToTrack[distanceToTrack > self.distanceToTrackTreshold] = 42
		distanceToCar = np.array(
			[np.linalg.norm(nearestPoints_obs[i] - nearestPoint_car) for i in range(len(nearestPoints_obs))])
		distanceToCar[distanceToCar < 0.1] = 44.0
		nearestObstacle = np.argmin(distanceToCar)

		if self.logging:
			print("nearestPoints_obs: ", nearestPoints_obs)
			print("nearestPoint_car: ", nearestPoint_car)
			print("distanceToTrack: ", distanceToTrack)
			print("distanceToCar: ", distanceToCar)
			print("nearestObstacle: ", nearestObstacle)
			print("distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold: ",
				  distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold)
		print('.... process_obstacles()_______')

		print("distanceToCar: ", distanceToCar)
		print("nearestObstacle: ", nearestObstacle)
		print("distanceToCar[nearestObstacle]: ", distanceToCar[nearestObstacle])
		print("distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold: ",
			  distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold)

		return distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold
