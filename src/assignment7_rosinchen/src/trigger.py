import numpy as np
import rospy 
from std_msgs.msg import Bool 




def callback():
	print("Grtriggert!")
	rospy.sleep(2)
	pub.publish(True)
	rospy.sleep(10)
	pub.publish(False)

rospy.init_node('Trigger')


pub = rospy.Publisher('/trigger_bool', Bool, queue_size=10)
callback()




rospy.spin()
