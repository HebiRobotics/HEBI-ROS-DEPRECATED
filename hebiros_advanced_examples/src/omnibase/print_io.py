
import numpy as np
import matplotlib as plt

import hebi
from time import sleep
import rospy
import geometry_msgs.msg 
from geometry_msgs.msg import Twist
import math

#import sys, pygame



#Setup Code, Run Once
stop_loop = False

lookup = hebi.Lookup()
sleep(2)

print('Modules found on network:')
group = lookup.get_group_from_names(['HEBI'], ['Mobile IO'])

if group is None:
	print('Group not found: Did you forget to set the module family and names above?')
	exit(1)

group_command = hebi.GroupCommand(group.size)


# Best practice is to allocate this once, not every loop iteration
group_feedback = hebi.GroupFeedback(group.size)

# This effectively limits the loop below to 200Hz
group.feedback_frequency = 200.0



while not (stop_loop):
	fbk = group_feedback
	group_feedback = group.get_next_feedback(reuse_fbk=group_feedback)
	if (group_feedback == None):
		group_feedback = fbk
		continue
	io_a = group_feedback.io.a
	io_b = group_feedback.io.b

	controlA = group_command.io.a


	scale =  io_a.get_float(1)
	print(scale)
	group_command.io.a.set_float(3,io_a.get_float(1));
	group_command.io.a.set_float(4,io_a.get_float(2));
	group_command.io.a.set_float(5,io_a.get_float(8));
	group_command.io.a.set_float(6,io_a.get_float(7));
	group.send_command(group_command)

  # ... read/use feedback object contents here.



