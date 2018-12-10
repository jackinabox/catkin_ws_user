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
from cv_bridge import CvBridge, CvBridgeError
#try:
#    import queue as Queue
#except ImportError:
#    import Queue as Queue

#import matplotlib
#matplotlib.use('Agg')
#from matplotlib import pyplot as plt

# roslib.load_manifest('my_package')
trigger = True
image_width = 640
k_p = -1
error_queue_size = 10
#errors = Queue.Queue(maxsize=error_queue_size)
errors = deque(maxlen=error_queue_size)

def waitForTrigger():
    while trigger == False:
        rospy.loginfo(
            "%s: No starting signal received. Waiting for message...",
            rospy.get_caller_id())
        rospy.sleep(0.1)

def waitForFirstError():
    while not rospy.is_shutdown() and len(list(errors)) == 0:
        rospy.loginfo(
            "%s: No initial error message received. Waiting for message...",
            rospy.get_caller_id())
        rospy.sleep(0.1)

def callbackError(msg):
    rospy.loginfo("new error: %f" % msg)
    errors.appendleft(msg)

def callbackTrigger(msg):
    rospy.loginfo("str(msg)")
    rospy.loginfo("\tstr(msg.data)")
    global trigger
    trigger = msg

def get_error_array():
    return np.array(list(errors))

def get_latest_error():
    arr = get_error_array()
    return arr[0]

def get_mean_error():
    arr = get_error_array()
    return np.mean(arr)

def steering_mapping_linear(val):
    if val == 0:
        return 90
    else:
        return int((val + image_width//2) / image_width * 180.0)

def control():
    err = get_latest_error()
    u_t = k_p * err
    steering = int(steering_mapping_linear(u_t))
    pub_steering.publish(steering)
    pub_logsteering.publish(steering)
    rospy.loginfo("error: %d -- steering: %d" % (err, steering))


rospy.init_node("controller", anonymous=True)

# create subscribers and publishers
sub_error = rospy.Subscriber("/line_param_offset", Float32, callbackError, queue_size=1)
sub_trigger = rospy.Subscriber("/trigger_bool", Bool, callbackTrigger, queue_size=100)

pub_steering = rospy.Publisher("steering", UInt8, queue_size=100)
pub_logsteering = rospy.Publisher("controller/info", String, queue_size=100)

# TEST
err = -200.0
u_t = k_p * err
rospy.loginfo("%d" % u_t)
steering = int(steering_mapping_linear(u_t))
rospy.sleep(2)
pub_steering.publish(steering)
pub_logsteering.publish(steering)
rospy.loginfo("error: %d -- steering: %d" % (err, steering))


#waitForTrigger()
waitForFirstError()
while trigger:
    control()


# spin() simply keeps python from exiting until this node is stopped
rospy.spin()



