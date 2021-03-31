#! /usr/bin/env	python
from __future__	import division,print_function
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty,	String ,Bool
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
import autonomous.msg
import actionlib
import tf


import math
import time
import os
import numpy as	np
import traceback
import csv
import sys
#import logging
import datetime

#############################################
# Logging
#############################################

time_str=str(datetime.datetime.now().strftime('%y%m%d%H%M%S'))
#logging.basicConfig(level=logging.DEBUG)
#logger = logging.getLogger()

filename = os.path.basename(__file__).split(".")[0]+"_"+time_str+".log"
#filename="abc.log"
cwdir = os.path.dirname(os.path.realpath(__file__))

today_folder = os.path.join(cwdir,time_str[:6])
experiment_folder = os.path.join(today_folder,"command")
command_folder = os.path.join(experiment_folder,time_str[6:])
all_folder = [today_folder,experiment_folder,command_folder]
for f in all_folder:
	if not os.path.exists(f):
		os.mkdir(f)
#handler = logging.FileHandler(path)
path= os.path.join(command_folder,filename)
logger = open(path,"w",0)
#handler.setFormatter(logging.Formatter("%(filename)s %(asctime)s %(levelname)s %(lineno)s %(message)s"))
#handler.setLevel(logging.DEBUG)
#logger.addHandler(handler)
counter = 0

def	stop():
	return	Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0))
def forward():
	return Twist(Vector3(0.2,0.0,0.0),Vector3(0.0,0.0,0.0))
def backward():
	return Twist(Vector3(-0.2,0.0,0.0),Vector3(0.0,0.0,0.0))
def left():
	return Twist(Vector3(0.0,0.1,0.0),Vector3(0.0,0.0,0.0))
def right():
	return Twist(Vector3(0.0,-0.1,0.0),Vector3(0.0,0.0,0.0))
def ccw():
	return Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,1.0))
def cw():
	return Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,-1.0))

def	odometry_callback(odom):
	global x
	global y
	global z
	global yaw_angle
	x=odom.pose.pose.position.x
	y=odom.pose.pose.position.y
	z=odom.pose.pose.position.z
	orientation_x=odom.pose.pose.orientation.x
	orientation_y=odom.pose.pose.orientation.y	
	orientation_z=odom.pose.pose.orientation.z	
	orientation_w=odom.pose.pose.orientation.w
	roll_angle,pitch_angle,yaw_angle=tf.transformations.euler_from_quaternion((orientation_x,
		orientation_y,orientation_z,orientation_w))

	yaw_angle=yaw_angle/np.pi*180.0



def	flight(goal):
	global counter
	now=rospy.Time.now()
	rate = rospy.Rate(5)
	duration = 0
	move=stop()
	if goal.order.x:
		print("forward/backward")
		duration = goal.order.x
		if goal.order.x>0:
			move = forward()
		else:
			move = backward()
	elif goal.order.y:
		print("left/right")
		duration = goal.order.y
		if goal.order.y>0:
			move = left()
		else:
			move = right()
	elif goal.order.yaw:
		print("ccw/cw")
		duration = goal.order.yaw
		if goal.order.yaw>0:
			move = ccw()
		else:
			move = cw()
	goal_string = " ".join(str(goal).replace("\n"," ").split())
	move_string = " ".join(str(move).replace("\n"," ").split())
	
	logger.write("{} [goal] {} - {} ".format(counter,datetime.datetime.now(),goal_string))
	logger.write("[move] {}\n".format(move_string))
	counter+=1
	print("action received: ",goal.order.x,goal.order.y,goal.order.yaw)		
	print("before: ",x,y,yaw_angle)	
	while rospy.Time.now() - now<=rospy.Duration(secs=abs(duration)):
		pub.publish(move)
		rate.sleep()
	print("after: ",x,y,yaw_angle)		
	pub.publish(stop())			


	# pub.publish(stop())
	# pub_land.publish(Empty())

def command_callback(goal):
	flight(goal)
	print("wait for another command")
	result.finish=True
	action.set_succeeded(result)

def flying_state_callback(data):
	if data.state == 0 and not state_start:
		print("landed")
		raw_input("Press any key to takeoff again")
		pub_takeoff.publish(Empty())
		rospy.sleep(5)
	

x,y,yaw_angle=0,0,0

rospy.init_node('command_steering',anonymous=True)
pub_takeoff=rospy.Publisher('/bebop/takeoff', Empty,queue_size=10)
pub_land=rospy.Publisher('/bebop/land', Empty,queue_size=10)
pub_reset=rospy.Publisher('/bebop/reset',Empty,queue_size=10)
pub=rospy.Publisher('/bebop/cmd_vel', Twist,queue_size=10)
pub_camera=rospy.Publisher('/bebop/camera_control',Twist,queue_size=10)
while pub_camera.get_num_connections()==0:
	print("CAMERA no connection for camera")
	rospy.sleep(1)
print("Connect to camera")
pub_camera.publish(Twist(Vector3(0,0,0),Vector3(0,-20,0)))
print("Camera orientation set")	
if sys.argv[1] == "0" or sys.argv[1] == "1":
	print("manual or semi")
	action = actionlib.SimpleActionServer('command1',autonomous.msg.CommandAction,\
								   execute_cb=command_callback,auto_start=False)
	
else:
	print("auto")
	action = actionlib.SimpleActionServer("command",autonomous.msg.CommandAction,\
										  execute_cb=command_callback,auto_start=False)

result = autonomous.msg.CommandResult()
action.start()


rospy.Subscriber("bebop/odom",Odometry,odometry_callback)
# ensure FlyingStateChanged will not be called until take off
state_start=True
rospy.Subscriber("bebop/states/ARDrone3/PilotingState/FlyingStateChanged",
			 Ardrone3PilotingStateFlyingStateChanged,flying_state_callback)
rospy.sleep(2)

try:
	raw_input("Press any key to takeoff")
	pub_takeoff.publish(Empty())
	rospy.sleep(5)
	state_start=False
	print("wait for input from user/bebop_brain")		
	while not rospy.is_shutdown():
		rospy.sleep(0.1)

except	Exception as e:
	print("type error: ",str(e))
	print(traceback.format_exc())
	pub.publish(stop())
	rospy.sleep(0.1)
	pub_land.publish(Empty())

finally:
	pub.publish(stop())
	rospy.sleep(0.1)
	pub_land.publish(Empty())
	logger.close()


