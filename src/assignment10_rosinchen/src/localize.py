#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from setup_values import Setup
from model_track import Track


setup = Setup()
logging = setup.logging
carID = setup.carID  # 5
laneID = setup.laneID  # 0
distance = setup.lookahead_distance  # 0.35
model = Track(laneID, logging)

location = np.array([])
location_corr = np.array([])


def callback(data):
    global location
    global location_corr
    x, y = data.pose.pose.position.x, data.pose.pose.position.y
    #print("position:       ", x, y)
    p_ahead = model.look_ahead((x, y), distance)
    #print("p_ahead", p_ahead)
    to_pub = Point(p_ahead[0], p_ahead[1], 0)
    # print("ahead:          ",to_pub.x,to_pub.y)
    # print("    ------------------")
    pub_target.publish(to_pub)


# location = np.append(location, ([x, y]))
# location_corr = np.append(location_corr, (look_ahead((x, y),laneID)))
# rospy.loginfo("x,y:",data)
# print(location)
# np.save("location.npy", location)
# np.save("nearest_point.npy", location_corr)
# rospy.sleep(1)

rospy.init_node("localize", anonymous=True)

# publish topic /target_point

# create subscribers and publishers
pub_target = rospy.Publisher("/target_point", Point, queue_size=1)
sub_pos = rospy.Subscriber("/localization/odom/" + str(carID), Odometry, callback, queue_size=1)
# print("hello")

rospy.spin()
