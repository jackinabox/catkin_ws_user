#!/usr/bin/env python
import numpy as np
import rospy 
from geometry_msgs.msg import Point
from std_msgs.msg import Float32 
from sensor_msgs.msg import Image

def gerade(m,b,x):
	return m*x+b

def abstand(m,b):
	yref = 320
	xref = 320
	
	ab = yref-gerade(m,b,xref)

	return ab


def callback(msg):
	rospy.loginfo("HAAAAAAAAAAALLLLLLOOOO")
	m = msg.x
	b = msg.y
	rospy.loginfo(abstand(m,b))
	pub.publish(abstand(m,b))


rospy.init_node('line_offset')
pub = rospy.Publisher('/line_param_offset', Float32, queue_size=10)
rospy.sleep(1)
rospy.Subscriber('/line_parameter', Point, callback, queue_size=10)

"""
Wenn der Abstand negativ ist dann nach rechts lenken, wenn positiv dann nach links!
""" 


rospy.spin()
