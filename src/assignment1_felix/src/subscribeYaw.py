#!/usr/bin/env python
#receives yaw Angle under topic and publishes it as string
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

from geometry_msgs.msg import Quaternion
 
def callback(raw_msg):
	yaw=raw_msg
	publisher.publish("I heard yaw: "+str(yaw))

rospy.init_node("subAndPub_yaw_Node")

publisher=rospy.Publisher("/yawString",String)
rospy.Subscriber("/yaw",Float32,callback)

rospy.spin()