#! /usr/bin/env python


import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError

import cv2
import time
import datetime
import multiprocessing

class Camera:

	def __init__(self,node_name='bebop/image_raw',saveVideo=False):
		self.saveVideo=saveVideo
		self.image=None
		# rospy.wait_for_message(node_name,Image)
		rospy.Subscriber(node_name,Image,self.callback_image)
		print("Subscribe to image: ",node_name)
		time.sleep(1)

		if self.saveVideo:
			self.out=self.create_video_output()
			
	def display(self):
		while not rospy.is_shutdown():
			if self.image is not None:
				cv2.imshow("Bebop",self.image)
				if self.saveVideo:
					self.out.write(self.image)
				k = cv2.waitKey(1)&0xff
				if k == ord('q'):
					cv2.destroyAllWindows()
					if self.saveVideo:
						self.out.release()
					break
				if k == ord('p'):
					print("image captured")
					self.capture(self.image)
	
	def capture(self,image):
		time_str=str(datetime.datetime.now().strftime('%y%m%d%H%M%S'))
		cv2.imwrite("measure_"+time_str+".jpg",image)

	def callback_image(self,data):
		self.image=CvBridge().imgmsg_to_cv2(data,"bgr8")

	def create_video_output(self):
		time_str=str(datetime.datetime.now().strftime('%y%m%d%H%M%S'))
		fourcc=cv2.VideoWriter_fourcc("M","J","P","G")
		return cv2.VideoWriter("output_"+time_str+".avi",fourcc,10,(640,368),True)



if __name__ == "__main__":

	rospy.init_node('camera',anonymous=True)
	pub_camera=rospy.Publisher('/bebop/camera_control',Twist,queue_size=10)
	while pub_camera.get_num_connections()==0:
		print("CAMERA no connection for camera")
		time.sleep(1)
	print("Connect to camera")
	pub_camera.publish(Twist(Vector3(0,0,0),Vector3(0,-20,0)))
	print("Camera orientation set")	
	c=Camera(saveVideo=False)
	c.display()
	
	
