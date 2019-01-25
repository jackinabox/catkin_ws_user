#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from setup_values import Setup
from model_track import Track
from obstacle_detector import Obstacle_detector

setup = Setup()
logging = setup.logging
carID = setup.carID  # 5
laneID = setup.laneID_initial  # 0
lookahead_distance = setup.lookahead_distance_initial  # 0.35
lookahead_distance_scale = setup.lookahead_distance_factor
threshold_obstacle_distance = setup.threshold_obstacle_distance
model = Track(laneID, logging)

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
		print("p_ahead", p_ahead)
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
	lookahead_distance = data * lookahead_distance_scale


def callback_avoid_obstacle(data):


def callback_lane_set_to(data):
	model.set_lane(data.data)


def callback_lane_switch(data):
	model.switch_lane()


# location = np.append(location, ([x, y]))
# location_corr = np.append(location_corr, (look_ahead((x, y),laneID)))
# rospy.loginfo("x,y:",data)
# print(location)
# np.save("location.npy", location)
# np.save("nearest_point.npy", location_corr)
# rospy.sleep(1)

rospy.init_node("driver", anonymous=True)
print(" ##### driver started ######")

# create subscribers and publishers
pub_target = rospy.Publisher("/target_point", Point, queue_size=1)
sub_pos = rospy.Subscriber("/localization/odom/" + str(carID), Odometry, callback_update_position, queue_size=1)
sub_curr_speed = rospy.Subscriber("/manual_control/speed", Int16, callback_update_lookahead_distance, queue_size=1)
sub_laser = rospy.Subscriber("/scan", LaserScan, callback_avoid_obstacle, queue_size=1)

# lane stuff
sub_lane_switch_to = rospy.Subscriber("/driver/lane_set_to", UInt8, callback_lane_set_to, queue_size=10)
sub_lane_switch = rospy.Subscriber("/driver/lane_switch", String, callback_lane_switch, queue_size=10)

rospy.spin()
