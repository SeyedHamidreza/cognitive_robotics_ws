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



# this is necessary for multi threading and client-server system
graph = tf.Graph()
with graph.as_default():
    config = tf.ConfigProto()
    #config.gpu_options.per_process_gpu_memory_fraction = 0.5
    config.gpu_options.allow_growth = True
    session = tf.Session(config=config)
    with session.as_default():    
        ### create the network model
        mobilenet_model = mobilenet_v2.MobileNetV2(weights='imagenet', include_top=True)
        mobilenet_feature_extraction = Model(inputs=mobilenet_model.input, outputs=mobilenet_model.get_layer('global_average_pooling2d_1').output)
        mobilenet_model._make_predict_function()
        #plot_model(mobilenet_model, to_file='model.png')
        print(mobilenet_model.summary())

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
    max_pooling = 1 #TRUE 

    if rospy.has_param('/perception/image_normalization'):
        image_normalization = rospy.get_param("/perception/image_normalization")
        #print ("########## image_normalization (0 : FALSE, 1 : TRUE) = " + str(image_normalization))

    if rospy.has_param('/perception/multiviews'):
        multiviews = rospy.get_param("/perception/multiviews")
        #print ("########## multiviews (0 : FALSE, 1 : TRUE) = " + str(multiviews))

    if rospy.has_param('/perception/max_pooling'):
        max_pooling = rospy.get_param("/perception/max_pooling")
        #print ("########## max_pooling (0 : FALSE, 1 : TRUE) = " + str(max_pooling))

    #__________________________
    #|                         |
    #|    MAIN SERVICE CODE    |
    #|_________________________|

    image_size = 224
    #normalization 
    #The mean pixel values are taken from the VGG authors, which are the values computed from the training dataset.
    mean_pixel = [103.939, 116.779, 123.68]

    #print (len(req.good_representation)/3)    
    nbin= int (math.sqrt(len(req.good_representation)/3))
    print ("number of bin = " + str(nbin))

    new_image = req.good_representation
    tic = time.clock()
    tic_global = time.clock()

    i=0
    b = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    b = np.reshape(b, (nbin, nbin))
    max_b = np.max(b)
    b = b * (255/max_b)


    # b = np.int16(b) # convert to signed 16 bit integer to allow overflow
    # b = np.clip(b, 0, 255) # force all values to be between 0 and 255
    # # after clip img2 is effectively unsigned 8 bit, but make it explicit:
    # eq_b = np.uint8(b)
    # b = cv2.equalizeHist(eq_b)

    i=i+1
    g = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    g = np.reshape(g, (nbin, nbin))
    max_g = np.max(g)
    g = g * (255/max_g)

    # g = np.int16(g) # convert to signed 16 bit integer to allow overflow
    # g = np.clip(g, 0, 255) # force all values to be between 0 and 255
    # # after clip img2 is effectively unsigned 8 bit, but make it explicit:
    # eq_g = np.uint8(g)
    # g = cv2.equalizeHist(eq_g)


    i=i+1
    r = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    r= np.reshape(r, (nbin, nbin))
    max_r = np.max(r)
    r = r * (255/max_r)

    # r = np.int16(r) # convert to signed 16 bit integer to allow overflow
    # r = np.clip(r, 0, 255) # force all values to be between 0 and 255
    # # after clip img2 is effectively unsigned 8 bit, but make it explicit:
    # eq_r = np.uint8(r)
    # r = cv2.equalizeHist(eq_r)

    
    # zero_image = np.zeros(nbin*nbin)
    # zerolayer = np.reshape(zero_image, (nbin, nbin))

    if (not multiviews): # make an image using three planes of good descriptor
        img_good = cv2.merge((b,g,r))
        resized_image_tmp = cv2.resize(img_good, (image_size, image_size), interpolation = cv2.INTER_AREA)
        resized_image = resized_image_tmp
        #cv2.imshow ('test',resized_image_tmp)
        #cv2.imwrite('/tmp/6.jpg', resized_image)

        #normalization 
        if (image_normalization):
            for c in range(3):
                resized_image[:, :, c] = resized_image[:, :, c] - mean_pixel[c]
        
        x = image.img_to_array(resized_image)
        x = np.expand_dims(x, axis=0)
        x = preprocess_input(x)
        toc = time.clock()
        print ("image preparation took " + str (toc - tic))
        tic = time.clock()

        with graph.as_default():  
            with session.as_default():    
                features = mobilenet_feature_extraction.predict(x)

    else: # make three independet images using good descriptor
        ### blue image 
        #img_b = cv2.merge((b,zerolayer,zerolayer))
        img_b = cv2.merge((b,b,b))
        resized_img_b = cv2.resize(img_b, (image_size, image_size), interpolation = cv2.INTER_AREA)

        if (image_normalization):
            #resized_img_b[:, :, 0] = resized_img_b[:, :, 0] - mean_pixel[0]
            for c in range(3):
                resized_img_b[:, :, c] = resized_img_b[:, :, c] - mean_pixel[c]

        x_b = image.img_to_array(resized_img_b)
        x_b = np.expand_dims(x_b, axis=0)
        x_b = preprocess_input(x_b)

        #cv2.imwrite('/tmp/1.jpg', resized_img_g)

        ### green image 
        #img_g = cv2.merge((g,zerolayer,zerolayer))
        img_g = cv2.merge((g,g,g))
        resized_img_g = cv2.resize(img_g, (image_size, image_size), interpolation = cv2.INTER_AREA)
        #cv2.imwrite('/tmp/2.jpg', resized_img_g)

        if (image_normalization):
            #resized_img_b[:, :, 1] = resized_img_b[:, :, 1] - mean_pixel[1]
            for c in range(3):
                resized_img_g[:, :, c] = resized_img_g[:, :, c] - mean_pixel[c]
            
        x_g = image.img_to_array(resized_img_g)
        x_g = np.expand_dims(x_g, axis=0)
        x_g = preprocess_input(x_g)

        ### red image 
        #img_r = cv2.merge((r,zerolayer,zerolayer))
        img_r = cv2.merge((r,r,r))
        resized_img_r = cv2.resize(img_r, (image_size, image_size), interpolation = cv2.INTER_AREA)
        #cv2.imwrite('/tmp/3.jpg', resized_img_r)

        if (image_normalization):
            #resized_img_b[:, :, 2] = resized_img_b[:, :, 2] - mean_pixel[2]
            for c in range(3):
                resized_img_r[:, :, c] = resized_img_r[:, :, c] - mean_pixel[c]
        
        x_r = image.img_to_array(resized_img_r)
        x_r = np.expand_dims(x_r, axis=0)
        x_r = preprocess_input(x_r)
        toc = time.clock()

        # plt.imshow (img_good)
        # plt.imshow (img_good[:,:,2])
        plt.show()

        with graph.as_default():  
            with session.as_default():    
                features_b = mobilenet_feature_extraction.predict(x_b)
                features_g = mobilenet_feature_extraction.predict(x_g)
                features_r = mobilenet_feature_extraction.predict(x_r)

        toc = time.clock()
        print ("deep learning took " + str (toc - tic))
            
        ### multi view pooling
        tic = time.clock()
        if (max_pooling) :
            features = np.max([features_b, features_g, features_r], axis=0)
        else:
            features = np.average([features_b, features_g, features_r], axis=0)
            #features = np.concatenate([features_b, features_g, features_r], axis=0)
        toc = time.clock()
        #print ("pooling took " + str (toc - tic))
        
        print ("size of representation is "+ str(len(features[0])))
        print ("size of representation is "+ str(features.shape))

        toc_global = time.clock()
        print ("########### Deep learning took " + str (toc_global - tic_global))
    
        
    return deep_representationResponse(features[0])

def mobilenetV2_server():
    rospy.init_node('deep_learning_representation_server')
    s = rospy.Service('mobilenetV2_service', deep_representation, handle_good_representation)
    print "Ready to representas based on mobilenetV2 network."
    rospy.spin()

if __name__ == "__main__":
    mobilenetV2_server()


