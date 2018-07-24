#!/usr/bin/env python

from __future__ import print_function

import rospy

import geometry_msgs.msg 
from geometry_msgs.msg import Twist

import sys, pygame


startMsg = """
Hello User! Lookin' good today
Reading from Keyboard, Publishing to keys/cmd_vel
CTRL-C to quit
"""

screenMsg = """
NAVIGATION INSTRUCTIONS

W : forward
S : back
Q : left
E : right
A : rotate left
D : rotate right

J : increase speed
K : reset speed to default
L : lower speed

P : quit

Enjoy!
"""

screenMsg = screenMsg.split("\n")

keyBindings = {
		'd':(0,0,1), #right
		'D':(0,0,1),#
		'a':(0,0,-1),
		'A':(0,0,-1),
		's':(0,-1,0),
		'S':(0,-1,0),
		'w':(0,1,0),
		'W':(0,1,0),
		## these are for the diagonal + rotational movement
		# 'q':(0,1,-1),
		# 'Q':(0,1,-1),
		# 'e':(0,1,1),
		# 'E':(0,1,1),
		## these are for the horizontal translation
		'q':(-1,0,0),
		'Q':(-1,0,0),
		'e':(1,0,0),
		'E':(1,0,0),
		##
	}

speedBindings = {
		'j': 1.1,
		'k': 1,
		'l': 0.9,	
	}

def shutdownHook():
	print ("Shutting Down!")

if __name__=="__main__":

	pub = rospy.Publisher('keys/cmd_vel', Twist, queue_size = 3)
	rospy.init_node('key_read_node')
	pygame.init()
	pygame.font.init()

	print(startMsg)
	screen = pygame.display.set_mode((600,600))

	x = 0
	y  = 0
	th = 0
	running = True
	n = len(screenMsg) + 1


	r = rospy.Rate(100)

	####### PARAMETERS ##########
	speed = 1
	turnspeed = 1

	while (not rospy.is_shutdown() and running):
		screen.fill((254,128,195))
		basicfont = pygame.font.SysFont(None, 38)
		count = 1

		for word in screenMsg:
			text = basicfont.render(word, True, (0, 0, 0))
			textrect = text.get_rect()
			textrect.centerx = screen.get_rect().centerx
			textrect.centery = count * screen.get_rect().height / n 
			screen.blit(text, textrect)
			count += 1

		pygame.display.flip()


		for event in pygame.event.get():
			#print(event)
			if event.type == pygame.QUIT:
				pygame.quit()
				exit()
			if event.type == pygame.KEYDOWN:
				#print(event.unicode)
				if event.key == pygame.K_p: # p to quit
					pygame.quit()
					running = False
					rospy.on_shutdown(shutdownHook)
				if event.unicode in keyBindings.keys():
					x = keyBindings[event.unicode][0]
					y = keyBindings[event.unicode][1]
					th = keyBindings[event.unicode][2]
				if (event.unicode in speedBindings.keys()):
					if event.key == pygame.K_k:
						speed = 1 # k acts as a reset button for the speed
					else:
						speed = speed * speedBindings[event.unicode]

			if event.type == pygame.KEYUP:
				x = 0
				y = 0
				z = 0
				th = 0

		twist = Twist()
		# important parameters
		twist.linear.x = x * speed; twist.linear.y = y * speed; 
		twist.angular.z = th*turnspeed;
		# not important parameters, NO NEED TO CHANGE
		twist.linear.z = 0;
		twist.angular.x = 0; twist.angular.y = 0; 
		#print(twist)
		pub.publish(twist)

		r.sleep()

	rospy.on_shutdown(shutdownHook)
	# rospy.spin()
