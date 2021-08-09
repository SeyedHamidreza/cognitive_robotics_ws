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
from keras.applications import inception_v3
from keras.applications.inception_v3 import InceptionV3
from keras.preprocessing import image
from keras.applications.inception_v3 import preprocess_input
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
    session = tf.Session()
    with session.as_default():    
        ### Load the model and include the "top" classification layer
        inception_model = inception_v3.InceptionV3(weights='imagenet', include_top=True)
        #inception_feature_extraction = Model(inputs=inception_model.input, outputs=inception_model.get_layer('avg_pool').output)
        inception_model._make_predict_function()
        plot_model(inception_model, to_file='model.png')
        print(inception_model.summary())


#__________________________________
#|                                 |
#|           GLOBAL VARS           |
#|_________________________________|

_name = 'deep_learning_services'
def handle_good_representation(req):
    
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
    
    
    i=i+1
    g = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    g = np.reshape(g, (nbin, nbin))
    max_g = np.max(g)
    g = g * (255/max_g)
    
    i=i+1
    r = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    r= np.reshape(r, (nbin, nbin))
    max_r = np.max(r)
    r = r * (255/max_r)
        
    ## VGG16, VGG19, and ResNet all accept 224 * 224 input images while Inception V3 and Xception require 299 * 299 pixel inputs
    # img_good = cv2.merge((b,g,r))
    # resized_image_tmp = cv2.resize(img_good, (299, 299), interpolation = cv2.INTER_AREA)
    # resized_image = resized_image_tmp
    
    #cv2.imshow ('test',resized_image_tmp)
    #cv2.imwrite('/tmp/6.jpg', resized_image)

    #normalization 
    # for c in range(3):
    #     resized_image[:, :, c] = resized_image[:, :, c] - mean_pixel[c]
    
    # x = image.img_to_array(resized_image)
    # x = np.expand_dims(x, axis=0)
    # x = preprocess_input(x)
    # toc = time.clock()
    # print ("image preparation took " + str (toc - tic))

    tic = time.clock()

    # make three independet images using good descriptor 
    # zero_image = np.zeros(nbin*nbin)
    # zerolayer = np.reshape(zero_image, (nbin, nbin))

    #img_b = cv2.merge((b,zerolayer,zerolayer))
    img_b = cv2.merge((b,b,b))
    resized_img_b = cv2.resize(img_b, (299, 299), interpolation = cv2.INTER_AREA)
    #resized_img_b[:, :, 0] = resized_img_b[:, :, 0] - mean_pixel[0]
    # for c in range(3):
    #     resized_img_b[:, :, c] = resized_img_b[:, :, c] - mean_pixel[0]

    x_b = image.img_to_array(resized_img_b)
    x_b = np.expand_dims(x_b, axis=0)
    x_b = preprocess_input(x_b)


    #img_g = cv2.merge((g,zerolayer,zerolayer))
    img_g = cv2.merge((g,g,g))
    resized_img_g = cv2.resize(img_g, (299, 299), interpolation = cv2.INTER_AREA)
    #resized_img_b[:, :, 1] = resized_img_b[:, :, 1] - mean_pixel[1]
    # for c in range(3):
    #     resized_img_g[:, :, c] = resized_img_g[:, :, c] - mean_pixel[1]

    x_g = image.img_to_array(resized_img_g)
    x_g = np.expand_dims(x_g, axis=0)
    x_g = preprocess_input(x_g)


    #img_r = cv2.merge((r,zerolayer,zerolayer))
    img_r = cv2.merge((r,r,r))
    resized_img_r = cv2.resize(img_r, (299, 299), interpolation = cv2.INTER_AREA)
    # resized_img_b[:, :, 2] = resized_img_b[:, :, 2] - mean_pixel[2]
    # for c in range(3):
    #     resized_img_r[:, :, c] = resized_img_r[:, :, c] - mean_pixel[2]

    x_r = image.img_to_array(resized_img_r)
    x_r = np.expand_dims(x_r, axis=0)
    x_r = preprocess_input(x_r)
    toc = time.clock()


    # plt.imshow (img_good)
    #plt.imshow (img_good[:,:,2])
    # plt.show()
    # TODO : fc2 should be change to avg_pool
    with graph.as_default():  
        with session.as_default():    
            #features = inception_feature_extraction.predict(x)
            ### multi view
            features_b = inception_feature_extraction.predict(x_b)
            features_g = inception_feature_extraction.predict(x_g)
            features_r = inception_feature_extraction.predict(x_r)

    toc = time.clock()
    print ("deep learning took " + str (toc - tic))
    #deep_representation_b = fc2_features_b.reshape((4096,1))
    #deep_representation_g = fc2_features_g.reshape((4096,1))
    #deep_representation_r = fc2_features_r.reshape((4096,1))

    ### multi view
    tic = time.clock()
    features = np.max([features_b, features_g, features_r], axis=0)
    toc = time.clock()
    #print ("pooling took " + str (toc - tic))
    
    print ("size of representation is "+ str(len(features[0])))

    toc_global = time.clock()
    print ("########### Deep learning took " + str (toc_global - tic_global))
   
    try: 
    #     #global global_counter
    #     #global_counter = global_counter + 1
          I =0
    #     plt.subplot(2, 1, 2)
    #     plt.grid(True)
          #plt.plot (fc2_features[0])
    #     plt.subplot(2, 2, 1)
    #     #plt.imshow (img_good)
    #     plt.subplot(2, 2, 2)
    #     #plt.imshow (resized_image)
    #     #plt.savefig('/home/hamidreza/Desktop/test/' + str(global_counter) + '.png')
         # plt.show()
    except ValueError: 
        print("can not show the plot")
    
    return deep_representationResponse(features[0])

def inception_server():
    rospy.init_node('deep_learning_representation_server')
    s = rospy.Service('inception_service', deep_representation, handle_good_representation)
    print "Ready to representas based on inception network."
    rospy.spin()

if __name__ == "__main__":
    inception_server()


