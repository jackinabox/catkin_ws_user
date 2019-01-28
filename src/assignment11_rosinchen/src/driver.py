#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from setup_values import Setup
from model_track import Track
from obstacle_detector import ObstacleDetector
from sensor_msgs.msg import LaserScan


class Handbrake:
	def __init__(self, initial_state, log=False):
		# Model - values in cm
		self.active = initial_state

	def release(self):
		self.active = False

	def tighten(self):
		self.active = True

	def toggle(self):
		self.active = (self.active + 1) % 2


setup = Setup()
logging = setup.logging
carID = setup.carID  # 5
laneID = setup.laneID_initial  # 0
lookahead_distance = setup.lookahead_distance_initial  # 0.35
lookahead_distance_scale = setup.lookahead_distance_factor
model = Track(laneID, logging)
obstacle_detector = ObstacleDetector(setup.threshold_obstacle_distance, logging)
handbrake_state = setup.handbrake
handbrake = Handbrake(handbrake_state)

location_current = None
location = np.array([])
location_corr = np.array([])


def update_target(data):
	global location
	global location_corr
	x, y = data.pose.pose.position.x + setup.gps_offset[0], data.pose.pose.position.y + setup.gps_offset[1]
	if logging:
		print("position:       ", x, y)
	p_ahead = model.look_ahead((x, y), lookahead_distance)
	if logging:
		print("p_ahead: ", p_ahead)
	to_pub = Point(p_ahead[0], p_ahead[1], 0)
	# print("ahead:          ",to_pub.x,to_pub.y)
	# print("    ------------------")
	pub_target.publish(to_pub)


def callback_update_position(data):
	update_target(data)
	global location_current
	location_current = data


def callback_update_lookahead_distance(data):
	global lookahead_distance
	lookahead_distance = float(data.data) * lookahead_distance_scale
	pub_lookahead_dist.publish(Float32(lookahead_distance))
	if logging:
		print("speed: %f -> lookaheaddist: %f" % (data.data, lookahead_distance))


def callback_avoid_obstacle(data):
	if obstacle_detector.detects_an_obstacle(data, location_current, model):
		model.switch_lane()


def callback_lane_set_to(data):
	model.set_lane(data.data)


def callback_lane_switch(data):
	model.switch_lane()

def callback_handbrake_tighten(data):
	global handbrake
	handbrake = True
	pub_handbrake.publish(Bool(True))

def callback_handbrake_release(data):
	global handbrake
	handbrake = False
	pub_handbrake.publish(Bool(False))


# location = np.append(location, ([x, y]))
# location_corr = np.append(location_corr, (look_ahead((x, y),laneID)))
# rospy.loginfo("x,y:",data)
# print(location)
# np.save("location.npy", location)
# np.save("nearest_point.npy", location_corr)
# rospy.sleep(1)

rospy.init_node("driver", anonymous=True)
print(" ##### driver started ######")

# location, planning
pub_target = rospy.Publisher("/driver/target_point", Point, queue_size=1)
sub_pos = rospy.Subscriber("/localization/odom/" + str(carID), Odometry, callback_update_position, queue_size=1)

# lookahead distance
sub_curr_speed = rospy.Subscriber("/manual_control/speed", Int16, callback_update_lookahead_distance, queue_size=1)
pub_lookahead_dist = rospy.Publisher("/driver/info/look_ahead_distance", Float32, queue_size=1)

# obstacle detection
sub_laser = rospy.Subscriber("/scan", LaserScan, callback_avoid_obstacle, queue_size=1)

# lane stuff
sub_lane_switch_to = rospy.Subscriber("/driver/lane_set_to", UInt8, callback_lane_set_to, queue_size=10)
sub_lane_switch = rospy.Subscriber("/driver/lane_switch", String, callback_lane_switch, queue_size=10)

# speed
pub_handbrake = rospy.Publisher("/driver/handbrake/state", Bool, queue_size=1)
sub_handbrake_tighten = rospy.Subscriber("/driver/handbrake/tighten", String, callback_handbrake_tighten, queue_size=1)
sub_handbrake_release = rospy.Subscriber("/driver/handbrake/release", String, callback_handbrake_release, queue_size=1)


rospy.spin()
