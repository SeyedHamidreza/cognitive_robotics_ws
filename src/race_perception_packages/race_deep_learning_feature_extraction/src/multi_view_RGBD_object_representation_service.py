#!/usr/bin/env python

#__________________________________
#|                                 |
#|           IMPORTS               |
#|_________________________________|

import roslib; roslib.load_manifest('race_deep_learning_feature_extraction')
import rospy
import random
from race_deep_learning_feature_extraction.srv import *
from nodelet.srv import *
import threading
import os
import subprocess


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

from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
from tensorflow import keras
from keras.applications import mobilenet_v2
from keras.applications.mobilenet import MobileNet
from keras.preprocessing import image
from keras.applications.mobilenet import preprocess_input
from keras.models import Model
from tensorflow.keras import layers
from keras import backend as K
from keras.utils import plot_model

from pytictoc import TicToc
import time

# this is needed to get rid of cpu warning AUX
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

from keras.models import load_model
from tensorflow import Graph, Session

np.set_printoptions(threshold=np.inf)


def preprocessingForOrthographicImages (img, image_size):
    
    '''
        Note: since images are sparse, we need to apply dilation and erosion
    '''
    
    resized_img = cv2.resize(img, (image_size, image_size), interpolation=cv2.INTER_AREA)
    resized_img = np.array(resized_img, dtype = np.uint8)

    # https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_filtering/py_filtering.html
    # img_resized_blur = cv2.blur(img_resized,(5,5))
    img_resized_blur = cv2.bilateralFilter(resized_img,5,15,15)
    # img_resized_blur = cv2.bilateralFilter(resized_img,5,75,75) modelNet

    kernel = np.ones((5,5),np.uint8)
    dilation = cv2.dilate(img_resized_blur, kernel, iterations=1)
    othographic_image = 255 - dilation

    return resized_img, othographic_image




# # this is necessary for multi threading and client-server system
# graph = tf.Graph()
# with graph.as_default():
#     config = tf.ConfigProto()
#     #config.gpu_options.per_process_gpu_memory_fraction = 0.5
#     config.gpu_options.allow_growth = True
#     session = tf.Session(config=config)
#     with session.as_default():    
#         ### create the network model
#         mobilenet_model = mobilenet_v2.MobileNetV2(weights='imagenet', include_top=True)
#         mobilenet_feature_extraction = Model(inputs=mobilenet_model.input, outputs=mobilenet_model.get_layer('global_average_pooling2d_1').output)
#         mobilenet_model._make_predict_function()
#         #plot_model(mobilenet_model, to_file='model.png')
#         print(mobilenet_model.summary())


recognition_network = "MobileNet"

# Load the Network.
graph = tf.Graph()
with graph.as_default():
    # config = tf.ConfigProto()
    config = tf.compat.v1.ConfigProto()
    config.gpu_options.allow_growth = True
    session = tf.compat.v1.Session(config=config)

    # this is necessary for multi threading and client-server system
    with session.as_default():

        ### create the network model for object recognition part
        if (recognition_network == "MobileNet"):
            mobilenet_model = mobilenet_v2.MobileNetV2(weights='imagenet', include_top=True)
            encoder = Model(inputs=mobilenet_model.input, outputs=mobilenet_model.get_layer('global_average_pooling2d_1').output)
            mobilenet_model._make_predict_function()
            #plot_model(mobilenet_model, to_file='model.png')
            print(mobilenet_model.summary())
        elif (recognition_network == "CNN_AE"):
            autoencoder = load_model('/home/hamidreza/rosbuild_ws/race_deep_learning_feature_extraction/src/deep_autoencoder_model.h5')
            encoder = Model(inputs=autoencoder.input, outputs=autoencoder.get_layer('flatten_1').output)
            print ("encoder architecture")
            encoder.summary()
              

tmp_img = 0

def handle_good_representation(req):

    #__________________________
    #|                         |
    #|     ROS PARAMETERS      |
    #|_________________________|

    ## NOTE: all parametres should be defined in launch files  
    ## RULE: 0 : FALSE, 1 : TRUE
    image_normalization = 0 #FALSE 
    multiviews = 1 #TRUE 
    pooling_function = "MAX" #TRUE 
    number_of_bins = 150
    if rospy.has_param('/perception/image_normalization'):
        image_normalization = rospy.get_param("/perception/image_normalization")
        #print ("########## image_normalization (0 : FALSE, 1 : TRUE) = " + str(image_normalization))

    if rospy.has_param('/perception/multiviews'):
        multiviews = rospy.get_param("/perception/multiviews")
        #print ("########## multiviews (0 : FALSE, 1 : TRUE) = " + str(multiviews))

    if rospy.has_param('/perception/pooling_function'):
        pooling_function = rospy.get_param("/perception/pooling_function")
        print ("########## pooling_function = " + str(pooling_function))
    
    if rospy.has_param('/perception/number_of_bins'):
        number_of_bins = rospy.get_param("/perception/number_of_bins")
        print ("########## number_of_bins = " + str(number_of_bins))


    #__________________________
    #|                         |
    #|    MAIN SERVICE CODE    |
    #|_________________________|

    number_of_views = int(len(req.good_representation) / (number_of_bins*number_of_bins))
    print ("number_of_views = " + str(number_of_views))
                   
    image_size = 224
    #normalization 
    #The mean pixel values are taken from the VGG authors, 
    # which are the values computed from the training dataset.
    mean_pixel = [103.939, 116.779, 123.68]

    all_images = req.good_representation
    tic = time.clock()
    tic_global = time.clock()

    ### deep feature vector of orthographic projections
    for i in range(0, number_of_views):
        
        img = all_images[i * number_of_bins * number_of_bins:(i + 1) * number_of_bins * number_of_bins]
        img = np.reshape(img, (number_of_bins, number_of_bins))
        max_pixel_value = np.max(img)
        img = img * (255 / max_pixel_value)

        resized_img, othographic_image = preprocessingForOrthographicImages(img, image_size)

        with graph.as_default():
            with session.as_default():
                ## We represent each image as a feature vector

                if (recognition_network == "MobileNet"):                  
                    image_size = 224
                    resized_img, othographic_image = preprocessingForOrthographicImages(img, image_size)

                    img_g = cv2.merge((othographic_image, othographic_image, othographic_image))
                    x_r = image.img_to_array(img_g)
                    x_r = np.expand_dims(x_r, axis=0)
                    x_r = preprocess_input(x_r)
                    feature = encoder.predict(x_r)
                
                elif (recognition_network == "CNN_AE"):
                    image_size = 64
                    resized_img, othographic_image = preprocessingForOrthographicImages(img, image_size)
                    othographic_image = othographic_image * 1. / 255
                    img = np.expand_dims(othographic_image, axis=-1)
                    feature = encoder.predict(img[None])
                
        # pooling functions
        if (i == 0):
            global_object_representation = feature            
        elif (pooling_function == "MAX"):
            global_object_representation = np.max([global_object_representation, feature], axis=0)
        elif (pooling_function == "AVG"):
            global_object_representation = np.average([global_object_representation, feature], axis=0)
        elif (pooling_function == "APP"):
            global_object_representation = np.append(global_object_representation, feature, axis=1)


    ### deep feature vector of rgb projections
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(req.RGB_image, "bgr8")

        cv2.imshow('image', cv_image)
        cv2.waitKey(1)
        print ("visualize image")

        with graph.as_default():
            with session.as_default():
                ## We represent each image as a feature vector

                if (recognition_network == "MobileNet"):                  
                    image_size = 224
                    resized_img, othographic_image = preprocessingForOrthographicImages(cv_image, image_size)

                    x_r = image.img_to_array(othographic_image)
                    x_r = np.expand_dims(x_r, axis=0)
                    x_r = preprocess_input(x_r)
                    feature = encoder.predict(x_r)
                
                elif (recognition_network == "other"):
                    print ("should be implemented")

    except CvBridgeError as e:
        print(e)
        print ("error visualize image")

      
    # pooling functions # size of feature can be check first and then do this part
    if (pooling_function == "MAX"):
        global_object_representation = np.max([global_object_representation, feature], axis=0)
    elif (pooling_function == "AVG"):
        global_object_representation = np.average([global_object_representation, feature], axis=0)
    elif (pooling_function == "APP"):
        global_object_representation = np.append(global_object_representation, feature, axis=1)

    toc = time.clock()
    print ("deep learning took " + str (toc - tic))
            
    toc = time.clock()
    #print ("pooling took " + str (toc - tic))
    
    print ("size of representation is "+ str(len(global_object_representation[0])))
    print ("size of representation is "+ str(global_object_representation.shape))

    toc_global = time.clock()
    print ("########### Deep learning took " + str (toc_global - tic_global))
            
    return deep_representationResponse(global_object_representation[0])

def RGBD_multiview_service():
    rospy.init_node('deep_learning_representation_server')
    s = rospy.Service('RGBD_multiview_service', deep_representation, handle_good_representation)
    print "Ready to representas RGBD object based on mobilenetV2 network."
    rospy.spin()

if __name__ == "__main__":
    RGBD_multiview_service()


