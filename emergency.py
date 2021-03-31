#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import time
import os

import pyxhook



def kbevent(event):
	global running
	global stop_bebop
	os.system('clear')
	print("To stop and land bebop, press X")
	print("To stop this program, press Z")
	# print("-",end="")
	# print(event.Ascii)

	# "X"
	if event.Ascii==120:
		print("stop and land")
		stop_bebop=True
	# "Z"
	if event.Ascii==122:
		print("exit")
		running=False

hookman=pyxhook.HookManager()
hookman.KeyDown=kbevent
hookman.HookKeyboard()
hookman.start()

rospy.init_node('emergency_shutdown',anonymous=True)
# pub_takeoff=rospy.Publisher('/bebop/takeoff', Empty,queue_size=10)
pub_land=rospy.Publisher('/bebop/land',Empty,queue_size=10)
pub=rospy.Publisher('/bebop/cmd_vel',Twist,queue_size=10)
# time.sleep(1)
# pub_takeoff.publish(Empty())

running=True
stop_bebop=False
print("To stop and land bebop, press X")
print("To stop this program, press Z")
while running:
	if stop_bebop:
		pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
		time.sleep(0.1)
		pub_land.publish(Empty())
		stop_bebop=False
	time.sleep(0.1)

# print("i am here")
pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
pub_land.publish(Empty())

hookman.cancel()

