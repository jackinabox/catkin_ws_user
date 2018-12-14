#!/usr/bin/env python

# --- imports ---

import rospy
import numpy as np
from std_msgs.msg import UInt8
from std_msgs.msg import Int16


import time

ticks = 0
startspeed = 300
zeitraum = 7

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

	
	
	print(final/finaltime)
	return final/finaltime


rospy.init_node("speed_function", anonymous=True)

# create subscribers and publishers
sub_ticks = rospy.Subscriber("/ticks", UInt8, appendingTicks, queue_size=1)
#sub_speed =  rospy.Subscriber("/manual_control/speed", Int16, queue_size=10)

#pub_rps = rospy.Publisher("rps", Int16, queue_size=1)

pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=1)



rospy.sleep(1)
pub_speed.publish(startspeed)
rospy.sleep(1)



countingTicks()

pub_speed.publish(0)

strecke = float(raw_input("Wie weit bist du gefahren?: "))

tickabstand = strecke/ticks

rospy.loginfo("Tickabstand:"+str(tickabstand))




# spin() simply keeps python from exiting until this node is stopped
rospy.spin()



#speed = 200
#weg  =3.97 m
#zeit = 12 Sekunden
#ticks/second = 51.6706016345, 51.1328548655
#meter/ticks= 0.006140625, 0.00618897637795 

#speed = 300
#zeit = 7 s
#Weg = 4.55, 4.47, 4.57
#ticks/second = 94.8427007499, 94.6961037053, 93.9095957436
#tickabstand = 0.00642655367232, 0.00626928471248, 0.00650071123755

#speed = 400
#zeit = 4s
#weg = 4.20
#ticks/second = 137.179777847,138.517866651, 137.604328429 
#tickabstand = 0.00658950617284, 0.00680685358255, 0.00658307210031


#ticks/sec(auto) = 0.433*auto-35.46666667

#Mit Mittelwerten!
#ticks/sec(auto) = 0.43182798*auto-34.99777655 
#meter/ticks = 0.0064381978570000001 m/ticks = 6.438197857 mm/tick
#m/s(auto) = (0.43182798*auto-34.99777655 )*0.0064381978570000001



