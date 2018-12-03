#!/usr/bin/env python
#receives scan data and calculates shortest distance to wall
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
#from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
 

def callback(raw_msg):
	rospy.loginfo("start callback")
	scan=raw_msg
	publisher.publish("I heard scan: "+str(scan))

rospy.init_node("node_wall_scan")

rospy.loginfo("start publish and subscribe")
pub_scan=rospy.Publisher("/wall_scan",String, queue_size=1)
rospy.Subscriber("/scan/LaserScan",LaserScan,callback)
rospy.loginfo("done.")

rospy.spin()