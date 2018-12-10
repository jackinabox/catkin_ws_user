import numpy as np
import rospy 
from std_msgs.msg import Bool 


def callback():
	rospy.loginfo("Go!")
	pub.publish(True)
	rospy.sleep(10)
	rospy.loginfo("Stop!")
	pub.publish(False)

rospy.init_node("Trigger", anonymous=True)


pub = rospy.Publisher('/trigger_bool', Bool, queue_size=10)
rospy.sleep(2)
callback()


rospy.spin()
