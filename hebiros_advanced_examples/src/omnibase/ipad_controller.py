#!/usr/bin/env python


import numpy as np
import matplotlib as plt
import hebi
from time import sleep
import rospy
import geometry_msgs.msg 
from geometry_msgs.msg import Twist

#import sys, pygame



#Setup Code, Run Once
lookup = hebi.Lookup()
sleep(2)

print('Modules found on network:')
group = lookup.get_group_from_names(['HEBI'], ['Mobile IO'])

if group is None:
	print('Group not found: Did you forget to set the module family and names above?')
	exit(1)


group_feedback = hebi.GroupFeedback(group.size)
#group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
group.feedback_frequency = 200.0
group_command = hebi.GroupCommand(group.size)

drive_state = False;


pub = rospy.Publisher('cmd_vel', Twist, queue_size = 3)
pub_rosie = rospy.Publisher('keys/cmd_vel', Twist, queue_size = 3)
rospy.init_node('key_read_node')


x = 0
y  = 0
th = 0
running = True

r = rospy.Rate(200)


####### PARAMETERS ##########
speed = 0
turnspeed = 1
threshold = 0.3
scale = 0
	
#Main Code
while (running):
	fbk = group_feedback
	group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
	#group_command = group.get_next_command(reuse_fbk=group_command)
	if (group_feedback == None):
		group_feedback = fbk
		continue

	#TODO

	io_a = group_feedback.io.a
	io_b = group_feedback.io.b
	controlA = group_command.io.a

	if(io_b.get_int(1) !=0):	
		drive_state = True;
		print("Drive Enabled")
	if(io_b.get_int(2) !=0):	
		drive_state = False;
		print("Drive Disabled")
	if(io_b.get_int(5) !=0):	
		scale = 0
		group_command.io.a.set_float(5,scale);
		group.send_command(group_command)
	if(io_b.get_int(8) !=0):	
		running = False;
		print("End of Program, disconnecting")

	


	x = io_a.get_float(7)
	y = io_a.get_float(2)
	th = io_a.get_float(1)
	scale = io_a.get_float(5)
	#controlA.set_float(5,scale)
	group_command.io.a.set_float(5,scale);
	group.send_command(group_command)

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
	
	twist_off.linear.x = 0; twist_off.linear.y = 0; twist_off.linear.z = 0;
	twist_off.angular.x = 0; twist_off.angular.y = 0; twist_off.angular.z = 0;

	if(drive_state):
		pub_rosie.publish(twist)
		r.sleep()
	else:
		pub_rosie.publish(twist_off)
		r.sleep()




#if __name__=="__main__":

group.feedback_frequency = 0.0
group.clear_feedback_handlers()

