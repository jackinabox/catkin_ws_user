#!/usr/bin/env python


class Setup:

	def __init__(self):
		self.lookahead_distance = 0.35  # distance = 0.35
		self.carID = 5
		self.target_speed = 450
		self.curve_angle = 30
		self.slowdown_curve = 0.66
		self.laneID = 0
		self.logging = False
		self.gps_offset = [-0.03, -0.015]
		# rospy.loginfo("initial setup:\n")
