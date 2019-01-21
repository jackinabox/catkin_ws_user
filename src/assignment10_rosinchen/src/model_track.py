#!/usr/bin/env python

import numpy as np


class Track:

    def __init__(self, initial_lane, logging):
        # Model - values in cm
        self.center_upper = [196, 215.5]
        self.center_lower = [405, 215.5]
        self.lines_inner = [[[196, 79], [405, 79]], [[196, 352], [405, 352]]]
        self.lines_outer = [[[196, 47], [405, 47]], [[196, 384], [405, 384]]]
        # lines = [left line, right line] with left line = [upper point, lower point]
        self.radius_inner = 136.5
        self.radius_outer = 168.5

        self.twoLines = np.array((self.lines_inner, self.lines_outer)) / 100.  # convert to meter
        self.twoRadii = np.array((self.radius_inner, self.radius_outer)) / 100.
        self.twoCenters = np.array((self.center_upper, self.center_lower)) / 100.

        self.current_lane = initial_lane

        self.logging = logging

    def switch_lane(self):
        curr_lane = self.current_lane
        self.current_lane = (curr_lane + 1) % 2
        if self.logging:
            print("switched to lane %d" % self.current_lane)

    def set_lane(self, new_lane):
        self.current_lane = new_lane
        if self.logging:
            print("set lane to ID: %d" % self.current_lane)
