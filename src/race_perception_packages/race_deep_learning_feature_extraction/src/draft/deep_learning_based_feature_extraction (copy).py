#!/usr/bin/env python

#__________________________________
#|                                 |
#|           IMPORTS               |
#|_________________________________|

import roslib; roslib.load_manifest('race_perception_msgs')
import rospy
import random
from race_perception_msgs.msg import CompleteRTOV
from nodelet.srv import *
import threading
import os
import subprocess
import roslib


#__________________________________
#|                                 |
#|           GLOBAL VARS           |
#|_________________________________|

_name = 'deep_learning_based_feature_extraction'


# _________________________________
#|                                 |
#|             CLASS               |
#|_________________________________|

class DeepLearningBasedFeatureExtraction:

    #Class variables


    def callback(data):

        #rospy.loginfo("%s: Deep learning feature extraction callback %s" % (_name, data.pipeline_number))
        subprocess.call(cmd_or,shell=True)
	print ('get data\n')

    ##
    # @brief
    # @return
    def __init__(self):
        #rospy.init_node(_name)

	#found = _name.rfind('/')
	#topic = _name[0:found]+"/object_descriptor/new_histogram_tracked_object_view"
	topic = "/perception/pipeline1/object_descriptor/new_histogram_tracked_object_view"
	print (topic)


        rospy.Subscriber(topic, CompleteRTOV, self.callback) #subscribe to all cycle messages from the nodes
    

        rospy.loginfo("%s: initialized" % _name)

    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':

    rospy.init_node(_name, anonymous=True)
    dl = DeepLearningBasedFeatureExtraction()
    dl.run()



    #rospy.spin() #start to spin

