#!/usr/bin/env python

#__________________________________
#|                                 |
#|           IMPORTS               |
#|_________________________________|

import roslib; roslib.load_manifest('race_deep_network_services')
import rospy
import random
from race_deep_network_services.srv import *
from nodelet.srv import *
import threading
import os
import subprocess

#__________________________________
#|                                 |
#|           GLOBAL VARS           |
#|_________________________________|


if __name__ == "__main__":

   rospy.wait_for_service('/vgg_service')
   vgg16_representation = rospy.ServiceProxy('/vgg_service', vgg16)
    
   good_representation = [1, 2, 3, 4, 5, 6]
   i=0
   for i in range (100000):
	    try:
		resp1 = vgg16_representation([1, 2, 3, 4, 5, 6])
		print (str (i) + str (resp1))
	    except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))



