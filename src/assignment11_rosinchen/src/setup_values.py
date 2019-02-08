#!/usr/bin/env python

class Setup:

	"""
	def __init__(self): # FAST SPEED  # -- you need to uncomment in driver.py
		self.lookahead_distance_initial = 0.50  # in meter
		self.lookahead_distance_factor = 0.00175#2  # % from manual speed
		self.carID = 5
		self.lidar_barcode_distance = 0.21
		self.target_speed = 750  # 0.50  # in m/s
		self.curve_angle = 25
		self.steering_offset = 5  # 125: 5; 127:
		self.slowdown_curve = 0.75
		self.threshold_distance_car_to_obstacle = 1.35
		self.threshold_detection_radius = 1.5  # in meter
		self.threshold_time_lane_switch = 0.1  # in sec
		self.laneID_initial = 1
		self.logging = False
		self.gps_offset = [-0.03, -0.015]
		self.handbrake = False
		# rospy.loginfo("initial setup:\n")
	"""
	def __init__(self): #OBSTACLE
		self.lookahead_distance_initial = 0.50  # in meter
		self.lookahead_distance_factor = 0.00133  # % from manual speed
		self.carID = 5
		self.lidar_barcode_distance = 0.21
		self.target_speed = 400  # 0.50  # in m/s
		self.curve_angle = 25
		self.steering_offset = 5  # 125: 5; 127:
		self.slowdown_curve = 0.85
		self.threshold_distance_car_to_obstacle = 1.35
		self.threshold_detection_radius = 1.5  # in meter
		self.threshold_time_lane_switch = 0.1  # in sec
		self.laneID_initial = 1
		self.logging = False
		self.gps_offset = [-0.03, -0.015]
		self.handbrake = False
		# rospy.loginfo("initial setup:\n")
	#"""