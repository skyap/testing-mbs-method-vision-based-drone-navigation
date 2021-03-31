#! /usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import tf

import pygame
import math
import time



class STEERING():
    move={
        # x,y,z,yaw
        pygame.K_UP:(1,0,0,0), # forward
        pygame.K_DOWN:(-1,0,0,0), # backward
        pygame.K_LEFT:(0,1,0,0), # left
        pygame.K_RIGHT:(0,-1,0,0), # right
        pygame.K_w:(0,0,1,0), # up
        pygame.K_s:(0,0,-1,0), # down
        pygame.K_a:(0,0,0,1), # rotate left
        pygame.K_d:(0,0,0,-1) # rotate right
    }   
    
    def __init__(self):
        self.x=0
        self.y=0
        self.z=0
        self.orientation_x=0
        self.orientation_y=0
        self.orientation_z=0
        self.orientation_w=0
        self.roll_angle=0
        self.pitch_angle=0
        self.yaw_angle=0


        self.pub_takeoff=rospy.Publisher('/bebop/takeoff', Empty,queue_size=10)
        self.pub_land=rospy.Publisher('/bebop/land', Empty,queue_size=10)
        self.pub_reset=rospy.Publisher('/bebop/reset',Empty,queue_size=10)

        self.pub=rospy.Publisher('/bebop/cmd_vel', Twist,queue_size=10)
        rospy.Subscriber('/bebop/odom',Odometry,self.callback)
        
        r=rospy.Rate(10)
        publisher=[self.pub_takeoff,self.pub_land,self.pub,self.pub_reset]

        print "Try to connect to all the listener"
        for i in publisher:
            while i.get_num_connections()==0:
                r.sleep()
        print "Connection established"
        rospy.on_shutdown(self.land)
        
    def takeoff(self):
        self.pub_takeoff.publish(Empty())

    def land(self):
        self.pub_land.publish(Empty())

    def emergency(self):
        self.pub_reset.publish(Empty())
        
    def direction(self,twist):
        self.pub.publish(twist)

    def callback(self,odom):

        # This contains the position of a point in free space
        # print data2.pose.pose.position.x  
        # This represents an orientation in free space in quaternion form        
        self.x=odom.pose.pose.position.x
        self.y=odom.pose.pose.position.y
        self.z=odom.pose.pose.position.z
        self.orientation_x=odom.pose.pose.orientation.x
        self.orientation_y=odom.pose.pose.orientation.y 
        self.orientation_z=odom.pose.pose.orientation.z 
        self.orientation_w=odom.pose.pose.orientation.w
        self.roll_angle,self.pitch_angle,self.yaw_angle=tf.transformations.euler_from_quaternion((self.orientation_x,self.orientation_y,self.orientation_z,self.orientation_w))
    
    def keyboardSteer(self):
        pygame.init()
        pygame.display.set_mode((100,100))
        pygame.key.set_repeat(1,100)

        stop=False
        direction=self.twist([0,0,0,0])
        status="land"
        print("Ready to take off")

        while not stop:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    if event.key in self.move.keys() and status=="take off":
                        direction=self.twist(self.move[event.key])
                        print direction
                    elif event.key == pygame.K_SPACE and status=="land":
                        print "take off"
                        status="take off"
                        self.takeoff()
                    elif event.key == pygame.K_x and status=="take off":
                        print "land"
                        status="land"
                        self.land()
                    elif event.key == pygame.K_ESCAPE and status=="land":
                        print "escape"
                        stop=True
                else:
                    if status=="land":
                        continue
                    else:
                        print "stop"
                        direction=self.twist([0,0,0,0])                
            self.direction(direction)
        pygame.quit()



    def __str__(self):
        return "\nx="+str(self.x)+ \
        "\ny="+str(self.y)+ \
        "\nz="+str(self.z)+ \
        "\norientation.x="+str(self.orientation_x)+ \
        "\norientation.y="+str(self.orientation_y)+ \
        "\norientation.z="+str(self.orientation_z)+ \
        "\norientation.w="+str(self.orientation_w)+ \
        "\nroll="+str(self.roll_angle)+ \
        "\npitch="+str(self.pitch_angle)+ \
        "\nyaw="+str(self.yaw_angle)
        
    def twist(self,command):
        x,y,z,yaw=command
        return Twist(Vector3(x,y,z),Vector3(0,0,yaw))
        
if __name__=="__main__":        
    rospy.init_node('teleop_twist_keyboard',anonymous=True)
    keyboard=STEERING()
    keyboard.keyboardSteer()

    