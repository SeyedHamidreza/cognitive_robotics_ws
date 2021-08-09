#!/usr/bin/env python

#__________________________________
#|                                 |
#|           IMPORTS               |
#|_________________________________|

### general
import roslib; roslib.load_manifest('race_deep_learning_feature_extraction')
import rospy
import random
import os
import sys
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt




# Just disables the warning, doesn't enable AVX/FMA :
#Your CPU supports instructions that this TensorFlow binary was not compiled to use: AVX2 AVX512F FMA
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'


### ROS
# Import custom message data and dynamic reconfigure variables.
from race_perception_msgs.msg import CompleteRTOV
#from race_deep_network_services.srv import *
from race_deep_learning_feature_extraction.srv import vgg16_model

from nodelet.srv import *
import threading
import subprocess
import roslib
# Give ourselves the ability to run a dynamic reconfigure server.
from dynamic_reconfigure.server import Server as DynamicReconfigureServer


### TensorFlow and tf.keras
# FOR INSTALL TensorFlow and keras use the followinig commands
# pip install --upgrade tensorflow-gpu
# pip install keras --user

import tensorflow as tf
from tensorflow import keras
from keras.applications import vgg16
from keras.applications.vgg16 import VGG16
from keras.preprocessing import image
from keras.applications.vgg16 import preprocess_input
from keras.models import Model
from tensorflow.keras import layers
from keras import backend as K

from pytictoc import TicToc

from keras.models import load_model
from tensorflow import Graph, Session


## check tf uses GPU or CPU   
#from tensorflow.python.client import device_lib
#device_lib.list_local_devices()

# this is necessary, otherwise this does not work
# It returns the default graph for the current thread.
# check https://www.tensorflow.org/api_docs/python/tf/get_default_graph

### create the network model
#vgg_model = vgg16.VGG16(weights='imagenet', include_top=True)
#vgg_feature_extraction = Model(inputs=vgg_model.input, outputs=vgg_model.get_layer('fc2').output)

#K.clear_session()

_name = 'deep_learning_representation'


#__________________________________
#|                                 |
#|           GLOBAL VARS           |
#|_________________________________|



# Node example class.
class DeepLearningNode():
    
    @staticmethod
    def load_network_model():
        vgg_model = vgg16.VGG16(weights='imagenet', include_top=True)
        vgg_feature_extraction = Model(inputs=vgg_model.input, outputs=vgg_model.get_layer('fc2').output)
        return vgg_feature_extraction
    
    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):
        # Initialize the node and name it.
        print ('python version = ' + str(sys.version_info[0]) + '.' + str(sys.version_info[1]))
        print('TF version = ' + str(tf.__version__))
        print('Keras version = ' + str(keras.__version__))
        # Get the ~private namespace parameters from command line or launch file.
        _name = rospy.get_name()
        _namespace = rospy.get_namespace()
        print ('node name = ' + str(_name))
        print ('name_space = ' + str(_namespace))
        self.timer_tic_toc = TicToc() #create instance of class
        
        
        # rosservice for object representation 
        rospy.wait_for_service('/vgg_service')
        self.vgg16_representation = rospy.ServiceProxy('/vgg_service', vgg16_model)
        
                
        #self.model = self.load_network_model()
        #self.graph = tf.get_default_graph()
        
        #rospy.wait_for_service('/vgg_service')
        #self.vgg16_representation = rospy.ServiceProxy('/vgg_service', vgg16_model, persistent=False)
        
        #try:
            #resp1 = self.vgg16_representation([0,1,1,2,1,43,5,6,7])
            #print resp1
        #except rospy.ServiceException as exc:
            #print("Service did not process request: " + str(exc))
            
       
        #self.graph = tf.Graph()
        #with self.graph.as_default():     
            #self.session = tf.Session(graph=self.graph)
            #with self.session.as_default():
                #### to get the feature layer we have to get access to the FC layer that is in top layers so, include_top=True
                #self.vgg_model = vgg16.VGG16(weights='imagenet', include_top=True)
                #self.vgg_feature_extraction = Model(inputs=self.vgg_model.input, outputs=self.vgg_model.get_layer('fc2').output)
        
                #fc2_features = self.vgg_feature_extraction.predict(x)
                
                
        
        
        #self.vgg_model = vgg16.VGG16(weights='imagenet', include_top=True)
        #self.vgg_feature_extraction = Model(inputs=self.vgg_model.input, outputs=self.vgg_model.get_layer('fc2').output)

        #self.vgg_model._make_predict_function()

        #blank_image = np.zeros((224,224,3), np.uint8)
        #x = image.img_to_array(blank_image)
        #x = np.expand_dims(x, axis=0)
        #x = preprocess_input(x)
        #self.vgg_feature_extraction.predict(x)

        #self.session = K.get_session()
        #self.graph = tf.get_default_graph()
        #self.graph.finalize() # this will make your graph read-only so it can be safely used in multiple threads. 
        

        rospy.loginfo("%s: initialized" % _name)
        # Create a publisher for our custom message.
        self.pub_topic = _namespace + "object_descriptor/deep_learning_representation_of_tracked_object_view"
        self.pub = rospy.Publisher(self.pub_topic, CompleteRTOV, queue_size=10)
        self.nbin= rospy.get_param("/perception/number_of_bins")
        
       
        # Create a listener for our custom message.
        self.topic = _namespace + "object_descriptor_deep_learning/new_histogram_tracked_object_view"
        rospy.Subscriber(self.topic, CompleteRTOV, self.callback) #subscribe to all cycle messages from the nodes

        # this topic is used to test with bringup_imperial.launch 
        self.topic_test = _namespace +"object_descriptor/new_histogram_tracked_object_view"
        print (self.topic)
        rospy.Subscriber(self.topic_test, CompleteRTOV, self.callback) #subscribe to all cycle messages from the nodes
       
        rospy.spin()

    def callback(self, data):
        
        #rospy.loginfo("I heard %s", data.sitov[0].spin_image)
        print ('################## deep learning package ##################\n')
        print ('################## deep learning package ##################\n')
        print ('################## deep learning package ##################\n')
        #rospy.loginfo("%s subscribe to:" % self._topic)
        #print (data.sitov[0].spin_image)
        rospy.loginfo("this view is a key_view  %d", data.is_key_view)
        
        new_image = data.sitov[0].spin_image
        self.timer_tic_toc.tic() #Start timer
        try:
            rep = self.vgg16_representation(data.sitov[0].spin_image)
            #print (rep)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        
        self.timer_tic_toc.toc(restart=True) #Time elapsed since t.tic() it took around 0.012870
        data.sitov[0].spin_image = rep.deep_representation
        self.pub.publish(data)


        #nbin= self.nbin
        ##print ('nbin =' + str(nbin))
        
    
        #i=0
        #b = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
        #b = np.reshape(b, (nbin, nbin))
        #b = b * 255
        ##print (b)

        #i=i+1
        ##g = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
        #g= np.zeros(nbin*nbin)
        #g = np.reshape(g, (nbin, nbin))
        #g = g * 255
        ##print (g)
        
        ##i=i+1
        ##r = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
        #r= np.zeros(nbin*nbin)
        #r= np.reshape(r, (nbin, nbin))
        #r= r * 255
        ##print (r)
        
        #img_good = cv2.merge((b,g,r))
        
        ##plt.imshow (img_good)
        ##plt.imshow (img_good[:,:,2])
        ##plt.show()
        
        
        #resized_image = cv2.resize(img_good, (224, 224))
        #x = image.img_to_array(resized_image)
        #x = np.expand_dims(x, axis=0)
        #x = preprocess_input(x)
        
        #rep = vgg16_representation(data.sitov[0].spin_image)
        #print (rep)
        
        # This is a workaround for asynchronous execution
        # https://www.tensorflow.org/api_docs/python/tf/get_default_graph
        #global graph  

        #with self.graph.as_default():     
            ##with self.session.as_default():
            #fc2_features = self.vgg_feature_extraction.predict(x)
    
        #with self.graph.as_default():
            #fc2_features = self.model.predict(x)
    
        ##with self.session.as_default():
            ##with self.graph.as_default():
                ##fc2_features = self.vgg_feature_extraction.predict(x)
                ###fc2_features = vgg_feature_extraction.predict(x)
                
        #fc2_features = fc2_features.reshape((4096,1))
        ##print (fc2_featuresfc2_features)
        #self.timer_tic_toc.toc(restart=True) #Time elapsed since t.tic() it took around 0.012870
        #data.sitov[0].spin_image = fc2_features
        #self.pub.publish(data)


            
    def run(self):   
        rospy.init_node(_name)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()


# Main function.
if __name__ == '__main__':
    
    rospy.init_node(_name)
    

    #good_representation = [1, 2, 3, 4, 5, 6]
    #i=0
    #for i in range (100000):
        #try:
            #resp1 = vgg16_representation([1, 2, 3, 4, 5, 6])
            #print (str (i) + str (resp1))
        #except rospy.ServiceException as exc:
            #print("Service did not process request: " + str(exc))


    
    
    
    #
        
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        DLmodel = DeepLearningNode()
    except rospy.ROSInterruptException: pass
    
    try:
        DLmodel.run()
    except rospy.ROSInterruptException: pass

    rospy.spin() #start to spin
    
 
    
