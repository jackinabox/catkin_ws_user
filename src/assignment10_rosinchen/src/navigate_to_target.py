#!/usr/bin/env python

# --- imports ---
import roslib
import rospy
import sys
import cv2
import numpy as np
from collections import deque
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from setup_values import Setup

setup = Setup()
carID = setup.carID  # 5
target_speed = setup.target_speed  # 300
curve_angle = setup.curve_angle  # 30
slow_curve = setup.slowdown_curve  # 0.66

print("I'm starting up!")

desired_position = np.array([1.96, 2.155])


def callback_position(data):
    global desired_position
    x, y, w, z = data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.orientation.w, data.pose.pose.orientation.z
    # print("x,y,w:",x,y,w)
    current_position = np.array([x, y])

    #print("CurrPos:", current_position)

    orientation_angle = 2 * np.arccos(w) * np.sign(z)

    orientation_vector = np.array([np.cos(orientation_angle), np.sin(orientation_angle)])

    #print("Despos: ", desired_position)
    #print("Orientation: ", orientation_vector)
    #print("Orientation_Angle:", orientation_angle)

    origin_vector = current_position + np.array([0.3 * np.cos(orientation_angle), 0.3 * np.sin(orientation_angle)])

    #print("Origin:", origin_vector)

    desired_direction = desired_position - origin_vector

    #print("DesDirect: ", desired_direction)
    #print("TEST: ", np.dot(orientation_vector, desired_direction) / (
    #        np.linalg.norm(orientation_vector) * np.linalg.norm(desired_direction)))

    steering_angle_temp = np.arccos(np.dot(orientation_vector, desired_direction) / (
            np.linalg.norm(orientation_vector) * np.linalg.norm(desired_direction)))
    # print("Steering: ",steering_angle_temp)
    # if steering_angle_temp <= np.pi:
    #	steering_angle_temp=
    steering_angle = (steering_angle_temp) / (np.pi) * 180

    orientation_vector = np.array([orientation_vector[0], orientation_vector[1], 0])
    desired_direction = np.array([desired_direction[0], desired_direction[1], 0])

    orientation = np.cross(orientation_vector, desired_direction)[2]

    # print(orientation)

    #print("SteeringAngle: ",steering_angle)#*np.sign(orientation))

    #steering_angle_final = 180 - (np.clip(steering_angle * np.sign(orientation), -90, 90) + 90)
    steering_angle_final = np.clip(steering_angle * np.sign(orientation)*(-2)+90, 0, 180)
    #print(180 - (steering_angle * np.sign(orientation) + 90))
    #print(steering_angle_final)
    #print(np.linalg.norm(desired_direction))
    #print(" ")
    pub_steering.publish(steering_angle_final)
    print("final steering angle",steering_angle_final," , publish "+str(np.round(steering_angle_final,2)))

    if steering_angle_final > 90 + curve_angle or steering_angle_final < 90 - curve_angle:
        pub_speed.publish(target_speed * slow_curve)
    else:
        pub_speed.publish(target_speed)


# rospy.sleep(1)


def callback_update_destiny(data):
    global desired_position
    desired_position = np.array([data.x, data.y])


rospy.init_node("desired_steering", anonymous=True)

# create subscribers and publishers
sub_pos = rospy.Subscriber("/localization/odom/" + str(carID), Odometry, callback_position, queue_size=1)
sub_des = rospy.Subscriber("/target_point", Point, callback_update_destiny, queue_size=1)

pub_steering = rospy.Publisher("steering", UInt8, queue_size=1)
pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
# pub_speed = rospy.Publisher("/speed", UInt8, queue_size=1)


rospy.spin()
