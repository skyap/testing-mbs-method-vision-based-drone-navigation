#! /usr/bin/env	python
"""
uint8 state_landed=0  # Landed state
uint8 state_takingoff=1  # Taking off state
uint8 state_hovering=2  # Hovering / Circling (for fixed wings) state
uint8 state_flying=3  # Flying state
uint8 state_landing=4  # Landing state
uint8 state_emergency=5  # Emergency state
uint8 state_usertakeoff=6  # User take off state. Waiting for user action to take off.
uint8 state_motor_ramping=7  # Motor ramping state (for fixed wings).
uint8 state_emergency_landing=8  # Emergency landing state. Drone autopilot has detected defective sensor(s). Only Yaw argument in PCMD is taken into account. All others flying commands are ignored.
"""
from __future__ import division,print_function
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged,\
				Ardrone3PilotingStateAttitudeChanged,\
				Ardrone3PilotingStateAltitudeChanged,\
				CommonCommonStateBatteryStateChanged

import pyxhook
def battery_callback(data):
	print("percent of battery: "+str(data.percent))
	
def state_callback(data):
	if data.state == 0:
		print("landed")
	elif data.state == 1:
		print("takingoff")
	elif data.state == 2:
		print("hovering")
	elif data.state == 3:
		print("flying")
	elif data.state == 4:
		print("landing")
	else:
		print(data.state)
		
def attitude_callback(data):
	print(data)
	print("roll = ",data.roll)
	print("pitch = ",data.pitch)
	print("yaw = ",data.yaw)
	print("-------------------")
previous_altitude = 0
def altitude_callback(data):
	global previous_altitude
	if abs(data.altitude - previous_altitude)<0.01:
		pass
	else:
		print("altitude: ",data.altitude)
		previous_altitude = data.altitude
	
def kbevent(event):
	if event.Ascii ==119:
		# "w" to move up
		pub_cmd.publish(Twist(Vector3(0,0,1),Vector3(0,0,0)))
	elif event.Ascii == 115:
		# "s" to move down
		pub_cmd.publish(Twist(Vector3(0,0,-1),Vector3(0,0,0)))
	elif event.Ascii == 32:
		# "spacebar" to takeoff
		pub_takeoff.publish(Empty())
	elif event.Ascii == 120:
		# "x" to land
		pub_land.publish(Empty())
	else:
		pub_cmd.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
	rospy.sleep(0.1)

		
rospy.init_node('read_my_state')
#rospy.Subscriber("bebop/states/ARDrone3/PilotingState/FlyingStateChanged",
#				 Ardrone3PilotingStateFlyingStateChanged,state_callback)
#rospy.Subscriber("bebop/states/ARDrone3/PilotingState/AttitudeChanged",
#				 Ardrone3PilotingStateAttitudeChanged,attitude_callback)
rospy.Subscriber("bebop/states/ARDrone3/PilotingState/AltitudeChanged",
				 Ardrone3PilotingStateAltitudeChanged,altitude_callback)
rospy.Subscriber("bebop/states/common/CommonState/BatteryStateChanged",
				 CommonCommonStateBatteryStateChanged,battery_callback)

pub_takeoff=rospy.Publisher('/bebop/takeoff', Empty,queue_size=10)
pub_land=rospy.Publisher('/bebop/land', Empty,queue_size=10)
pub_cmd=rospy.Publisher('/bebop/cmd_vel', Twist,queue_size=10)	
	
hookman = pyxhook.HookManager()
hookman.KeyDown = kbevent
hookman.HookKeyboard()
hookman.start()

while not rospy.is_shutdown():
	rospy.sleep(0.1)

hookman.cancel()