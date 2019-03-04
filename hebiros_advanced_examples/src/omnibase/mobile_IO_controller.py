#!/usr/bin/env python

"""
This program allows for the Mobile IO application to work with a Hebi omnibase device on the same network

Commands:
B1: Enable Drive
B2: Disable Drive
B5: Reset Speed Scale
B8: Exit Program

@author Sami Mian < sami @ hebirobotics.com >
@since 2 Feb 2019

Installation Requirements:
This python code required the HEBI python API to be installed on the host machine in order to run.
You can install the API using the terminal command: pip install --user hebi-py
Please see the following link for more information: http://docs.hebi.us/tools.html#python-api

"""

import numpy as np
import hebi
from time import sleep
import rospy
import geometry_msgs.msg 
from geometry_msgs.msg import Twist


#Setup Code, Run Once
lookup = hebi.Lookup()
sleep(2)

group = lookup.get_group_from_names(['HEBI'], ['Mobile IO'])


if group is None:
	print('Group not found: Did you forget to set the module family and names above?')
	exit(1)
else:
	print("Mobile IO device found")

#hebi feedback settings/parameters
group_feedback = hebi.GroupFeedback(group.size)
group.feedback_frequency = 200.0
group_command = hebi.GroupCommand(group.size)



#ROS Publisher setup
pub_rosie = rospy.Publisher('/cmd_vel', Twist, queue_size = 3)
rospy.init_node('key_read_node')
r = rospy.Rate(200)

#Logic Vairables
x = 0
y  = 0
th = 0
running = True
drive_state = False;


####### PARAMETERS ##########
speed = 0
turnspeed = 1
threshold = 0.3
scale = 0
	
#Main Code
while (running):
	fbk = group_feedback
	group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
	if (group_feedback == None):
		group_feedback = fbk
		continue

	io_a = group_feedback.io.a
	io_b = group_feedback.io.b
	controlA = group_command.io.a

	if(io_b.get_int(1) !=0 and drive_state == False):	
		drive_state = True;
		print("Drive Enabled")
	if(io_b.get_int(2) !=0 and drive_state == True):	
		drive_state = False;
		print("Drive Disabled")
	if(io_b.get_int(5) !=0):	
		scale = 0
		group_command.io.a.set_float(5,scale);
		group.send_command(group_command)
	 	#print("Scale reset to 0, speed set to default")
	if(io_b.get_int(8) !=0):	
		running = False;
		print("End of Program, disconnecting from Mobile IO")

	x = io_a.get_float(7)
	y = io_a.get_float(2)
	th = io_a.get_float(1)
	scale = io_a.get_float(5)
	
	#read A5 value and adjust gui appropriately
	group_command.io.a.set_float(5,scale);
	group.send_command(group_command)

	#speed and value tuning for smoother drive
	speed = (scale + 2)/2
	if(abs(x)<threshold):
		x=0;
	if(abs(y)<threshold):
		y = 0;
	if(abs(th)<threshold):
		th = 0;

	twist = Twist()
	twist_off = Twist()
	# important parameters
	twist.linear.x = x * speed; twist.linear.y = y * speed; 
	twist.angular.z = th*turnspeed;
	# not important parameters, NO NEED TO CHANGE
	twist.linear.z = 0;
	twist.angular.x = 0; twist.angular.y = 0; 
	#parameters for 0 velocity
	twist_off.linear.x = 0; twist_off.linear.y = 0; twist_off.linear.z = 0;
	twist_off.angular.x = 0; twist_off.angular.y = 0; twist_off.angular.z = 0;

	if(drive_state):
		pub_rosie.publish(twist)
		r.sleep()
	else:
		pub_rosie.publish(twist_off)
		r.sleep()

#Close handlers on exit
group.feedback_frequency = 0.0
group.clear_feedback_handlers()

