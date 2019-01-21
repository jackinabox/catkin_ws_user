#!/usr/bin/env python
import rospy


class Setup:

    def __init__(self):
        self.lookahead_distance = 0.35  # distance = 0.35
        self.carID = 5
        self.target_speed = 450
        self.curve_angle = 30
        self.slowdown_curve = 0.66
        self.laneID = 0
        # rospy.loginfo("initial setup:\n")
