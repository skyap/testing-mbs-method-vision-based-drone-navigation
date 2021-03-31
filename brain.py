#!/usr/bin/env python
from __future__ import division,print_function
from collections import deque
import random
import numpy as np
import cv2
import datetime
import os
import copy

import rospy
import actionlib
import autonomous.msg
from sensor_msgs.msg import	Image
from cv_bridge import CvBridge
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged,\
						   CommonCommonStateBatteryStateChanged
from analysis import Analysis
from decision import Decision

import point 


#############################################
# rospy topic actions related
#############################################
rospy.init_node('bebop_brain',anonymous=True)
image_raw_bebop = None
print("waiting for bebop/image_raw topic")
rospy.wait_for_message('/bebop/image_raw',Image)

def callback_image(data):
	global image_raw_bebop
	image_raw_bebop = CvBridge().imgmsg_to_cv2(data,"bgr8")
rospy.Subscriber('/bebop/image_raw',Image,callback_image)

battery = 0
def battery_callback(data):
	global battery
	battery = data.percent
rospy.Subscriber("bebop/states/common/CommonState/BatteryStateChanged",
				 CommonCommonStateBatteryStateChanged,battery_callback)

print("publish to bebop/image_raw topic")
pub_image = rospy.Publisher('visual_info_image',Image,queue_size=10)

print("setting up action server")
command = actionlib.SimpleActionClient('command',autonomous.msg.CommandAction)
while not command.wait_for_server(rospy.Duration(2)):
	print("waiting for the bebop actions server to come up")
print("subscribe to bebop actions server")

def feedback_cb(feedback):
	print(feedback)
	
rospy.sleep(1)
#############################################
# default actions
#############################################

actions = {"forward":point.Point(1,0,0),
		  "backward":point.Point(-1,0,0),
		  "turn_ccw":point.Point(0,0,0.5),
		  "turn_cw":point.Point(0,0,-0.5),
		  "turn_ccw_half":point.Point(0,0,0.25),
		  "turn_cw_half":point.Point(0,0,-0.25),
		  "fly_over":point.Point(4,0,0)}
#############################################
# program related
#############################################
move_backward=False
select_direction = False

#############################################
# reset parameters before bebop restart
#############################################
def flying_state_callback(data):
	global select_direction
	global move_backward
	if data.state==0 and not state_start:
		print("landed")
		raw_input("Press ENTER to re-start> ")
		move_backward=False
		select_direction = False
		rospy.sleep(0.1)
state_start=True
rospy.Subscriber("bebop/states/ARDrone3/PilotingState/FlyingStateChanged",
			 Ardrone3PilotingStateFlyingStateChanged,flying_state_callback)


height = 368
width = 640


# wait for user input to start
raw_input("Press ENTER to start> ")
state_start=False
#############################################
# beginning
#############################################
analysis = Analysis()
decision = Decision()
while not rospy.is_shutdown():
	# get image from bebop
	if image_raw_bebop is None:
		continue
	else:
		image_raw = image_raw_bebop.copy()
	
	#############################################
	# make analysis
	#############################################
	pub_image.publish(CvBridge().cv2_to_imgmsg(image_raw,"bgr8"))
	
	hline,final_image = analysis.make_analysis(image_raw)
	print(hline)
	#############################################
	# make decision
	#############################################
	direction = decision.make_decision(hline)
	cv2.putText(final_image,direction,(10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,0),1,cv2.LINE_AA)
	#############################################
	# publish image
	#############################################
	pub_image.publish(CvBridge().cv2_to_imgmsg(final_image,"bgr8"))	

	#############################################
	# Action
	#############################################
	
	
	
	# loop through all actions
	# send actions to command steer	
	# wait for feedback
	print("direction:",direction,actions[direction])
	for i in range(1):
		# goal = autonomous.msg.CommandGoal(order = point.Point(3,0,0) )
		goal = autonomous.msg.CommandGoal(order = actions[direction])
		command.send_goal(goal,feedback_cb=feedback_cb)
		command.wait_for_result()
	
	image_raw = None
	rospy.sleep(1)
	

