#!/usr/bin/env python


class Setup:

	def __init__(self):
		self.lookahead_distance_initial = 0.45  # in meter
		self.lookahead_distance_factor = 0.1  # % from manual speed
		self.carID = 5
		self.target_speed = 450  # 0.50  # in m/s
		self.curve_angle = 25
		self.slowdown_curve = 0.80
		self.threshold_obstacle_distance = 0.5
		self.laneID_initial = 1
		self.logging = True
		self.gps_offset = [-0.03, -0.015]
		# rospy.loginfo("initial setup:\n")
