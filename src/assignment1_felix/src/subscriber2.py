#!/usr/bin/env python
#receives every (string) message under topic myMessageOutB and publishes it again under myMessageOut2B
import rospy
from std_msgs.msg import String

def callback(raw_msg):
	publisher.publish("I heard: "+str(raw_msg))

rospy.init_node("subAndPub_message_node")

publisher=rospy.Publisher("/myMessageOut2",String)
rospy.Subscriber("/myMessageOut",String,callback)

rospy.spin()