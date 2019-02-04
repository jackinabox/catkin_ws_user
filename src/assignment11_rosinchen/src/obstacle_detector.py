#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np


# import matplotlib.pyplot as plt


class ObstacleDetector:

	def __init__(self, threshold_distance_car_to_obstacle, threshold_detection_radius, lidar_barcode_distance, logging):
		self.threshold_distance_car_to_obstacle = threshold_distance_car_to_obstacle
		self.threshold_distance_obstacle_to_track = 0.1
		self.threshold_detection_radius = threshold_detection_radius
		self.lidar_correction = lidar_barcode_distance
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
		datenauto = np.zeros((360, 2)) + np.nan
		for inx, i in enumerate(daten):
			if (inx > 60) or (inx > 300):
				continue
			elif i > self.threshold_detection_radius:
				continue
			inxr = inx * np.pi / 180
			xauto = i * np.cos(inxr)
			yauto = i * np.sin(inxr)
			auto_vector = np.array([xauto + self.lidar_correction, yauto, 0, 1]).reshape(4, 1)
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
		#datenauto[np.isinf(datenauto)] = 42
		#datenauto[np.isnan(datenauto)] = 43

		# print("datenauto: ",datenauto)
		datenauto[np.isinf(datenauto)] = np.nan

		return datenauto

	def process_obstacles(self, obstacles, model, position):
		# print('_____process_obstacles() .....')
		x, y = position.pose.pose.position.x, position.pose.pose.position.y
		# car_lane = model.current_lane
		nearestPoints_obs = np.array([model.nearest_point(obs)[1] for obs in obstacles])
		nearestPoint_car = model.nearest_point([x, y])[1]
		# print("nearestPoints_obs: ", nearestPoints_obs)
		distanceToTrack = np.array(
			[np.linalg.norm(nearestPoints_obs[i] - obstacles[i]) for i in range(len(nearestPoints_obs))])
		distanceToCar = np.array(
			[np.linalg.norm(nearestPoints_obs[i] - nearestPoint_car) for i in range(len(nearestPoints_obs))])
		
		#distanceToCar[distanceToCar       > self.threshold_distance_car_to_obstacle] = np.nan
		#nearestPoints_obs[distanceToCar   > self.threshold_distance_car_to_obstacle] = np.nan
		distanceToTrack[distanceToTrack   > self.threshold_distance_obstacle_to_track] = np.nan
		nearestPoints_obs[distanceToTrack > self.threshold_distance_obstacle_to_track] = np.nan


		#distanceToCar[distanceToCar < 0.3] = 44.0
		try:
			nearestObstacle = np.nanargmin(distanceToCar)
		except ValueError:
			return False
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
		

		#print(distanceToCar[nearestObstacle])
		#datenauto[np.isinf(datenauto)] = np.nan
		##datenauto[np.isnan(datenauto)] = 43
		#plt.cla()
		##print(distanceToCar[nearestObstacle])
		#plt.scatter(nearestPoint_car[0],nearestPoint_car[1],marker="H",color="k")
		#plt.scatter(datenauto[:,0],datenauto[:,1],color="r")
		#plt.scatter(nearestPoints_obs[:,0],nearestPoints_obs[:,1],color="g")
		#plt.plot(current_position[0],current_position[1],marker="H")
		#plt.show(block=False)


		#---new start
		#minimum_number_of_obs_points=4
		#nearestObstacleS = np.argpartition(distanceToCar, minimum_number_of_obs_points)[0:minimum_number_of_obs_points]
		#isObstacle = distanceToCar[nearestObstacleS] < self.threshold_distance_car_to_obstacle
		#print("isObstacle:",np.sum(isObstacle))
		#return np.sum(isObstacle)==minimum_number_of_obs_points
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
