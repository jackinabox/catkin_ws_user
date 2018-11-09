#!/usr/bin/env python
#publishes a simple message with "hello world2" under topic "myMessageOut2"

import rospy
from std_msgs.msg import String

rospy.init_node("publish_message_node")
publisher=rospy.Publisher("/myMessageOut",String)

while not rospy.is_shutdown():
	my_message="hello_world"
	publisher.publish(my_message)
	rospy.sleep(0.5)
