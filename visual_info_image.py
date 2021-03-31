#! /usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import sys
import datetime
import os


def callback(data):
	global time_str
	global cwdir
	global running_number
	global image_number
	global info_folder
	global ori_folder

	image=CvBridge().imgmsg_to_cv2(data,"bgr8")	
	if running_number%2==1:
		cv2.namedWindow("visual_info_image")
		cv2.moveWindow("visual_info_image",100,200)
		cv2.imshow("visual_info_image",image)
		cv2.waitKey(1)

	if sys.argv[1]=="True":
		# file_name = "measure_"+time_str+"_"+str(running_number)+".jpg"

		# file_name = "measure_%s_%03d.jpg"%(time_str,running_number)
		file_name = "measure_%03d.jpg"%(image_number)
		if running_number%2==1:
			path = os.path.join(info_folder,file_name)
			image_number+=1
		else:
			path = os.path.join(ori_folder,file_name)
		cv2.imwrite(path,image)
		running_number+=1

if __name__=='__main__':
	time_str=str(datetime.datetime.now().strftime('%y%m%d%H%M%S'))
	cwdir = os.path.dirname(os.path.realpath(__file__))

	print("time stamp:",time_str)
	print("current working directory:",cwdir)

	# create folder
	today_folder = os.path.join(cwdir,time_str[:6])
	experiment_folder = os.path.join(today_folder,time_str[6:])
	ori_folder = os.path.join(experiment_folder,"ori")
	info_folder = os.path.join(experiment_folder,"info")
	all_folder = [today_folder,experiment_folder,ori_folder,info_folder]
	for f in all_folder:
		if not os.path.exists(f):
			os.mkdir(f)


	running_number=0
	image_number = 0
	rospy.init_node('visual_info_image_node',anonymous=True)
	rospy.Subscriber('visual_info_image',Image,callback)
	print("visual_info_image subscribe to visual_info_image")
	print("Display visual_info_image")

	while not rospy.is_shutdown():
		rospy.sleep(0.1)

	cv2.destroyAllWindows()

