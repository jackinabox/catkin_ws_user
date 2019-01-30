#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np


# import matplotlib.pyplot as plt


class ObstacleDetector:

	def __init__(self, threshold_distance_car_to_obstacle, threshold_detection_radius, logging):
		self.threshold_distance_car_to_obstacle = threshold_distance_car_to_obstacle
		self.threshold_distance_obstacle_to_track = 0.15
		self.threshold_detection_radius = threshold_detection_radius
		self.logging = logging

	def detects_an_obstacle(self, lidar, position, model):
		obstacles = self.rotate(lidar, position)
		return self.process_obstacles(obstacles, model, position)

	def rotate(self, lidar, position):
		#print("_____rotate() ....")

		x, y, w, z = position.pose.pose.position.x, position.pose.pose.position.y, position.pose.pose.orientation.w, position.pose.pose.orientation.z
		# current_position = np.array([x, y])
		orientation_angle = 2 * np.arccos(w) * np.sign(z)
		T = np.array([[np.cos(orientation_angle), -np.sin(orientation_angle), 0, x],
					  [np.sin(orientation_angle), np.cos(orientation_angle), 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])

		daten = lidar.ranges
		datenauto = np.zeros((360, 2))
		for inx, i in enumerate(daten):
			if (inx > 35) or (inx > 335):
				continue
			if i > self.threshold_detection_radius:
				continue
			inxr = inx * np.pi / 180
			xauto = i * np.cos(inxr)
			yauto = i * np.sin(inxr)
			auto_vector = np.array([xauto, yauto, 0, 1]).reshape(4, 1)
			#if self.logging:
			#	print("rotate() -> auto_vector.shape: ", auto_vector.shape)
			world_vector = np.dot(T, auto_vector)
			#if self.logging:
			#	print("rotate() -> world_vector.shape: ", world_vector.shape)
			# print(world_vector[:2,0].shape)
			# does not work: datenauto = np.append(datenauto,world_vector[:2,0].reshape(2,1),axis=1)
			datenauto[inx, :] = world_vector[:2, 0].reshape(1, 2)
			#if self.logging:
			#	print("rotate() -> world_vector[:2, 0].reshape(1, 2): ", world_vector[:2, 0].reshape(1, 2))
			#if self.logging:
			#	print("rotate() -> datenauto: ", datenauto.shape)
		# plt.scatter(world_vector[0],world_vector[1])
		# plt.show()
		#print("....rotate()_______")
		datenauto[np.isinf(datenauto)] = 42
		datenauto[np.isnan(datenauto)] = 43

		# print("datenauto: ",datenauto)
		return datenauto

	def process_obstacles(self, obstacles, model, position):
		# print('_____process_obstacles() .....')
		x, y = position.pose.pose.position.x, position.pose.pose.position.y
		# car_lane = model.current_lane
		nearestPoints_obs = [model.nearest_point(obs)[1] for obs in obstacles]
		nearestPoint_car = model.nearest_point([x, y])[1]
		# print("nearestPoints_obs: ", nearestPoints_obs)
		distanceToTrack = np.array(
			[np.linalg.norm(nearestPoints_obs[i] - obstacles[i]) for i in range(len(nearestPoints_obs))])
		distanceToTrack[distanceToTrack > self.threshold_distance_obstacle_to_track] = 42
		distanceToCar = np.array(
			[np.linalg.norm(nearestPoints_obs[i] - nearestPoint_car) for i in range(len(nearestPoints_obs))])
		distanceToCar[distanceToCar < 0.3] = 44.0
		nearestObstacle = np.argmin(distanceToCar)
		'''
		if self.logging:
			print("nearestPoints_obs: ", nearestPoints_obs)
			print("nearestPoint_car: ", nearestPoint_car)
			print("distanceToTrack: ", distanceToTrack)
			print("distanceToCar: ", distanceToCar)
			print("nearestObstacle: ", nearestObstacle)
			print("distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold: ",
				  distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold)
		'''
		

		#---new start
		minimum_number_of_obs_points=4
		nearestObstacleS = np.argpartition(distanceToCar, minimum_number_of_obs_points)[0:minimum_number_of_obs_points]
		isObstacle = distanceToCar[nearestObstacleS] < self.threshold_distance_car_to_obstacle
		#print("isObstacle:",np.sum(isObstacle))
		return np.sum(isObstacle)==minimum_number_of_obs_points
		#new end -------------



		# print('.... process_obstacles()_______')

		# print("distanceToCar: ", distanceToCar)
		# print("nearestObstacle: ", nearestObstacle)
		print("distanceToCar[nearestObstacle]: ", distanceToCar[nearestObstacle])
		#print("distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold: ",
		#	  distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold)
		if distanceToCar[nearestObstacle] < self.threshold_distance_car_to_obstacle:
			print("# OBSTACLE in %fm #" % distanceToCar[nearestObstacle])
			return True  # distanceToCar[nearestObstacle] < self.distanceToObstacleTreshold
		else:
			return False
