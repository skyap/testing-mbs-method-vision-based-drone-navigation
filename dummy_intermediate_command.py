#! /usr/bin/env python

from __future__ import division,print_function

import rospy
import actionlib
import autonomous.msg

import time
import sys
#import logging
import os
import datetime


# receive goal from bebop_brain and forward to command_steering
# user provide manual command if it is not correct
# provide a way to reduce the speed of execution
# receive result from command_steering and forward to bebop_brain

time_str=str(datetime.datetime.now().strftime('%y%m%d%H%M%S'))
#logging.basicConfig(level=logging.DEBUG)
#logger = logging.getLogger()

filename = os.path.basename(__file__).split(".")[0]+"_"+time_str+".log"
cwdir = os.path.dirname(os.path.realpath(__file__))

today_folder = os.path.join(cwdir,time_str[:6])
experiment_folder = os.path.join(today_folder,"dummy")
dummy_folder = os.path.join(experiment_folder,time_str[6:])
all_folder = [today_folder,experiment_folder,dummy_folder]
for f in all_folder:
	if not os.path.exists(f):
		os.mkdir(f)

path= os.path.join(dummy_folder,filename)
logger = open(path,"w",0)
#handler = logging.FileHandler(path)

#handler.setFormatter(logging.Formatter("%(filename)s %(asctime)s %(levelname)s %(lineno)s %(message)s"))
#handler.setLevel(logging.DEBUG)
#logger.addHandler(handler)
counter=0

def feedback_cb(feedback):
	print(feedback)
	

def semi():
	def execute_cb(goal):
		global counter
		print("goal from bebop_brain> ", goal)
		user_input=raw_input("Press ENTER to forward goal> ")
		if not user_input:
			print("goal forward to command_steering")
			print(str(goal))
			logger.write("{} [goal] {} - {}\n".format(counter,datetime.datetime.now(),str(goal).replace("\n"," ")))
			counter+=1
			command1.send_goal(goal,feedback_cb=feedback_cb)
			command1.wait_for_result()
			result.finish=True
			command.set_succeeded(result)
		else:
			print("forward user input to command_steering")
			point = autonomous.msg.Point()
			point.x,point.y,point.yaw=map(float,user_input.split())
			user_goal = autonomous.msg.CommandGoal(order=point)
			logger.write("{} [goal] {} - {}\n".format(counter,datetime.datetime.now(),str(goal).replace("\n"," ")))
			counter+=1
			command1.send_goal(user_goal,feedback_cb=feedback_cb)
			command1.wait_for_result()
			result.finish=True
			command.set_succeeded(result)
			
	rospy.init_node("dummy_intermediate_command")
	result = autonomous.msg.CommandResult()
	command = actionlib.SimpleActionServer('command',autonomous.msg.CommandAction,\
										   execute_cb=execute_cb,auto_start=False)
	command.start()
	
	command1 = actionlib.SimpleActionClient("command1",autonomous.msg.CommandAction)
	while not command1.wait_for_server(rospy.Duration(2)):
		print("waiting for the command_steering to come up")
	print("subscribe to bebop actions server")
	
	while not rospy.is_shutdown():
		rospy.sleep(0.1)

def manual():
	rospy.init_node("dummy_intermediate_command")
	command1 = actionlib.SimpleActionClient("command1",autonomous.msg.CommandAction)
	while not command1.wait_for_server(rospy.Duration(2)):
		print("waiting for the command_steering to come up")
	print("subscribe to bebop actions server")
	
	while not rospy.is_shutdown():
		user_input = raw_input("user_input: ")
		point = autonomous.msg.Point()
		if user_input:
			point.x,point.y,point.yaw=map(float,user_input.split())
		else:
			point.x=1
			point.y=0
			point.yaw=0
		user_goal = autonomous.msg.CommandGoal(order=point)
		command1.send_goal(user_goal,feedback_cb=feedback_cb)
		command1.wait_for_result()

def auto():
	pass

if sys.argv[1]=="0":
	manual()
elif sys.argv[1]=="1":
	semi()
elif sys.argv[1]=="2":
	auto()
		
logger.close()





