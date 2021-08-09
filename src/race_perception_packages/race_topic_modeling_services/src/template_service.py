#!/usr/bin/env python

#__________________________________
#|                                 |
#|           IMPORTS               |
#|_________________________________|

import roslib; roslib.load_manifest('race_topic_modeling_services')
import rospy
import random
from race_topic_modeling_services.srv import *
from nodelet.srv import *
import threading
import os
import subprocess


### general
import rospy
import random
import os
import sys
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt

from pytictoc import TicToc
import time

def handle_HDP_representation_and_recognition(req):

    #__________________________
    #|                         |
    #|     ROS PARAMETERS      |
    #|_________________________|

    ## NOTE: all parametres should be defined in launch files  
    ## RULE: 0 : FALSE, 1 : TRUE
    
    #example:
    image_normalization = 0 #FALSE 
        
    if rospy.has_param('/perception/image_normalization'):
        image_normalization = rospy.get_param("/perception/image_normalization")
        #print ("########## image_normalization (0 : FALSE, 1 : TRUE) = " + str(image_normalization))

    #__________________________
    #|                         |
    #|    MAIN SERVICE CODE    |
    #|_________________________|


    BoW_representation = req.bag_of_words_representation
    #print (BoW_representation)
    ground_truth_label =  req.ground_truth_label
    print ("ground_truth_label = " + ground_truth_label)
    teach_flag = req.teach_data
    # teach or test data
    print ("teach data = " + str(teach_flag))

    ### your code should be added here 	    
    


    ### making a response message first and then fill its fields; finally return it
    result = topic_modellingResponse()
    result.topic_representation = (1,2,2,1,1,1) 
    result.recognition_result = "test"   
	
    return result

def local_HDP_server():
    rospy.init_node('topic_modelling_local_HDP_server')
    s = rospy.Service('local_HDP_service', topic_modelling, handle_HDP_representation_and_recognition)
    # This declares a new service named local_HDP_service with the topic_modelling service type. All requests are passed to handle_HDP_representation_and_recognition function. 
    # handle_HDP_representation_and_recognition is called with instances of AddTwoIntsRequest and returns instances of AddTwoIntsResponse.
    print "*** Ready to represent and recognize a given 3D object based on Local-HDP algorithm ***"
    rospy.spin()

if __name__ == "__main__":
    local_HDP_server()


