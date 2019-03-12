#!/usr/bin/env python


import numpy as np
import hebi
from time import sleep
import rospy
import geometry_msgs.msg 
from geometry_msgs.msg import Twist




from time import sleep
lookup = hebi.Lookup()
# Give the Lookup process 2 seconds to discover modules
sleep(2)
print('Modules found on network:')
print(lookup.entrylist)
for entry in lookup.entrylist:
  print('{0} | {1}'.format(entry.family, entry.name))