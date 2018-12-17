#!/usr/bin/env python

# --- imports ---
#import roslib
import rospy
#import sys
#import cv2
import numpy as np
#from collections import deque
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
#from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image


trigger = True
u_init = 1.0 # initial car speed
k_p = 0.1 # P controller
k_d = 0.1 # D controller

actual_speed = None
actual_speed_dif = None
target_speed = 0.25 # in m/s
target_speed_dif = 0. # in m*s^-2

def waitForTrigger():
    while trigger == False:
        rospy.loginfo(
            "%s: No starting signal received. Waiting for message...",
            rospy.get_caller_id())
        rospy.sleep(0.5)

def waitForFirstSpeed():
    while actual_speed == None or actual_speed_dif == None:
        rospy.loginfo(
            "%s: No initial speed message received. Waiting for message...",
            rospy.get_caller_id())
        rospy.sleep(0.5)

def callback_actualSpeed(msg):
    global actual_speed
    actual_speed = msg.data

def callback_actualSpeedDif(msg):
    global actual_speed_dif
    actual_speed_dif = msg.data

def callbackTrigger(msg):
    rospy.loginfo("str(msg)")
    rospy.loginfo("\tstr(msg.data)")
    global trigger
    trigger = msg

#def get_error_array():
#    return np.array(list(errors))

#def get_latest_error():
#    arr = get_error_array()
#    return arr[0]

#def get_mean_error():
#    arr = get_error_array()
#    return np.mean(arr)

def speed_mapping(target): # input [m/s], output [car speed (~300)]
    return 359.687*target+81.0456

def control():
    global actual_speed
    global actual_speed_dif
    global target_speed
    global target_speed_dif

    speed = actual_speed
    speed_dif = actual_speed_dif
    #rospy.loginfo("sub_actualSpeed: %f, %f" % speed, speed_dif)
    u_t = target_speed + k_p * (target_speed-actual_speed) + k_d * (target_speed_dif-actual_speed_dif)
    speed_car = np.clip(speed_mapping(u_t), 0, 500)
    rospy.loginfo("publish speed"+str(speed_car))
    pub_speed.publish(speed_car)
    pub_logspeed.publish(str(speed_car))
    #rospy.loginfo("\terror: %d -- steering: %d" % (err, steering))
    rospy.sleep(0.1)

rospy.init_node("controller_speed", anonymous=True)

# create subscribers and publishers
sub_actualSpeed = rospy.Subscriber("/mps", Float32, callback_actualSpeed, queue_size=1)
sub_actualSpeed_dif = rospy.Subscriber("/mps_diff", Float32, callback_actualSpeedDif, queue_size=1)
sub_trigger = rospy.Subscriber("/trigger_bool", Bool, callbackTrigger, queue_size=10)

pub_speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=1)
pub_logspeed = rospy.Publisher("controller_speed/info", String, queue_size=1)

waitForTrigger()
waitForFirstSpeed()
while trigger and not rospy.is_shutdown():
    control()

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()



