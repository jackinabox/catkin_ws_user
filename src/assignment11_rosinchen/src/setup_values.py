#!/usr/bin/env python

class Setup:

	def __init__(self):
		self.lookahead_distance_initial = 0.50  # in meter
		self.lookahead_distance_factor = 0.001  # % from manual speed
		self.carID = 5
		self.target_speed = 300  # 0.50  # in m/s
		self.curve_angle = 25
		self.slowdown_curve = 0.85
		self.threshold_distance_car_to_obstacle = 1.0
		self.threshold_detection_radius = 1.5  # in meter
		self.threshold_time_lane_switch = 2  # in sec
		self.laneID_initial = 0
		self.logging = False
		self.gps_offset = [-0.03, -0.015]
		self.handbrake = False
		# rospy.loginfo("initial setup:\n")
