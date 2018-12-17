#m/s(auto) = (0.43182798*auto-34.99777655 )*0.0064381978570000001

#!/usr/bin/env python

# --- imports ---

import rospy
import numpy as np
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32

from collections import deque
import numpy as np

import time

ticks = 0
zeitraum = 0.5
past_queue_size = 5
past = deque(maxlen=past_queue_size)



def appendingTicks(msg):
	global ticks 
	ticks += msg.data

def countingTicks():	
	global ticks

	starttime = time.time()
	ticks = 0

	while time.time()-starttime<=zeitraum:
		#rospy.loginfo("Tickabstand:"+str(ticks))
		final = ticks
		finaltime = time.time()-starttime 
		rospy.sleep(0.01)

	
	
	#print(final/finaltime)
	return final/finaltime

def get_past_array():
    	return np.array(list(past))

def get_latest_past():
    	arr = get_past_array()
    	return arr[0]

def get_mean_past():
    	arr = get_past_array()
	return np.mean(arr)

def get_diff_past():
	arr = get_past_array()
	grad = np.gradient(arr)
	mean = np.mean(grad)
	return mean
	


rospy.init_node("velocity", anonymous=True)

# create subscribers and publishers
sub_ticks = rospy.Subscriber("/ticks", UInt8, appendingTicks, queue_size=1)
pub_mps = rospy.Publisher("mps", Float32, queue_size=1)
pub_mps_diff = rospy.Publisher("mps_diff", Float32, queue_size=1)

#Main



while not rospy.is_shutdown():
	
	tps=countingTicks()
	mps = tps*0.006438197857
	past.appendleft(mps)
	#rospy.loginfo("past:"+str(past))
	#print(len(get_past_array()))
	if len(get_past_array())>2:
		diff = get_diff_past()
		pub_mps_diff.publish(diff)
		rospy.loginfo("diff:"+str(diff))

	rospy.loginfo("mps:"+str(mps))

	pub_mps.publish(mps)
	






# spin() simply keeps python from exiting until this node is stopped
rospy.spin()







