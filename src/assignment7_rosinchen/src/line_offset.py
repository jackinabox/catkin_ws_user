import numpy as np
import rospy 
from geometry_msgs.msg import Point
from std_msgs.msg import Float32 



def gerade(m,b,x):
	return m*x+b

def abstand(m,b):
	yref = 320
	xref = 320
	
	ab = yref-gerade(m,b,xref)

	return ab


def callback(msg):
	
	m = msg.x
	b = msg.y
	print(m,b)	
	print(abstand(m,b))
	
	pub.publish(abstand(m,b))

rospy.init_node('line_offset')
rospy.Subscriber('/line_parameter', Point,callback, queue_size=10)

pub = rospy.Publisher('/line_offset', Float32, queue_size=10)


"""
Wenn der Abstand negativ ist dann nach rechts lenken, wenn positiv dann nach links!
""" 


rospy.spin()
