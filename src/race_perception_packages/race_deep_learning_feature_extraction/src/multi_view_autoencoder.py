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

from numpy import zeros, newaxis

import tensorflow as tf
from tensorflow import keras

from keras.preprocessing import image
from keras.applications.xception import preprocess_input
from keras.models import Model
from tensorflow.keras import layers
from keras import backend as K
from keras.utils import plot_model

from keras.models import load_model

from pytictoc import TicToc
import time

# this is needed to get rid of cpu warning AUX
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

from keras.models import load_model
from tensorflow import Graph, Session
from keras.layers import Dense, Flatten, Reshape, Input, InputLayer
from keras.models import Sequential
from keras import regularizers

np.set_printoptions(threshold=np.inf)

def build_autoencoder(img_shape, code_size):
    #https://stackabuse.com/autoencoders-for-image-reconstruction-in-python-and-keras/
    # The encoder
    # second_layer_size = code_size*2
    encoder = Sequential()
    encoder.add(InputLayer(img_shape))
    encoder.add(Flatten())
    encoder.add(Dense(code_size))
    # encoder.add(Dense(code_size, activation='relu', activity_regularizer=regularizers.l1(10e-5)))
    # encoder.add(Dense(second_layer_size, activity_regularizer=regularizers.l1(10e-5)))
    # encoder.add(Dense(code_size, activity_regularizer=regularizers.l1(10e-5)))

    # The decoder
    decoder = Sequential()
    decoder.add(InputLayer((code_size,)))
    # decoder.add(Dense(second_layer_size, activation='sigmoid'))
    decoder.add(Dense(np.prod(img_shape))) # np.prod(img_shape) is the same as 32*32*3, it's more generic than saying 3072
    decoder.add(Reshape(img_shape))

    return encoder, decoder

def encode_given_image(img, encoder):
    print (len(img))
    print(img[None].shape)
    """Draws original, encoded and decoded images"""
    # img[None] will have shape of (1, 100, 100, 1) which is the same as the model input
    # code = encoder.predict(img[None])[0]

    object_representation = encoder.predict(img[None])
    print('code[None].shape = ' + str(code[None].shape))
    print('code.shape' + str(code.shape))

    return object_representation

# def visualize(img, encoder, decoder):
#     print (len(img))
#     print(img[None].shape)
#     """Draws original, encoded and decoded images"""
#     # img[None] will have shape of (1, 100, 100, 1) which is the same as the model input
#     # code = encoder.predict(img[None])[0]
#
#     code = encoder.predict(img[None])
#     print('code[None].shape = ' + str(code[None].shape))
#     print('code.shape' + str(code.shape))
#
#     reconstructed = decoder.predict(code)
#
#     print('reconstructed.shape' + str(reconstructed.shape))
#
#     plt.subplot(1,3,1)
#     plt.title("Original")
#     img_original = cv2.merge((img, img, img))
#     print(img_original.shape)
#     plt.imshow(img_original)
#
#     plt.subplot(1,3,2)
#     plt.title("Code")
#     plt.imshow(code.reshape([code.shape[-1]//5,-1])) ## 50 X 10
#
#     plt.subplot(1,3,3)
#     plt.title("Reconstructed")
#     # plt.imshow(reco)
#     img_reconstructed = cv2.merge((reconstructed, reconstructed, reconstructed))
#     print(img_original.shape)
#     plt.imshow(img_original)
#     plt.show()

def visualize(img, encoder, decoder):
    print (len(img))
    print(img[None].shape)
    print ('content of the image =')
    print(img)

    """Draws original, encoded and decoded images"""
    # img[None] will have shape of (1, 100, 100, 1) which is the same as the model input
    # code = encoder.predict(img[None])[0]

    code = encoder.predict(img[None])
    print ('code[None].shape = ' + str(code[None].shape))
    print ('code.shape' + str(code.shape))
    print ('content of the code =')
    print (code)
    reconstructed = decoder.predict(code)

    print('reconstructed_image.shape' + str(reconstructed.shape))

    plt.subplot(1,3,1)
    plt.title("Original")
    img_original = cv2.merge((img, img, img))
    # print(img_original.shape)
    plt.imshow(img_original)

    plt.subplot(1,3,2)
    plt.title("Code")
    plt.imshow(code.reshape([code.shape[-1]//5,-1])) ## 50 X 10

    plt.subplot(1,3,3)
    plt.title("Reconstructed")
    test = reconstructed[0]
    print (test.shape)
    # plt.imshow(reco)
    img_reconstructed = cv2.merge((test, test, test))
    plt.imshow(img_reconstructed)
    plt.show()

## should be a parameter
size_of_object_representation = 100
contrast = 8# [1-10]

# img_shape should be the same as (100,100,1) since we have trained the network using this image size
img_dimensions = (100, 100, 1)
print ('image shape =' + str(img_dimensions))

# this is necessary for multi threading and client-server system
compressed_representation = 100
graph = tf.Graph()
with graph.as_default():
    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.5
    session = tf.Session(config=config)
    with session.as_default():
        # loading encoder and decoder model
        img_shape = (100, 100, 1)
        encoder, decoder = build_autoencoder(img_shape, compressed_representation)
        input_layer = Input(img_shape)
        code = encoder(input_layer)
        reconstruction = decoder(code)
        autoencoder = Model(input_layer, reconstruction)
        autoencoder.load_weights('/home/hamidreza/rosbuild_ws/race_deep_learning_feature_extraction/src/autoencoder_model.h5')
        print ("encoder architecture")
        encoder.summary()
        print ("decoder architecture")
        decoder.summary()



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

    image_size = 100
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
    top = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    top = np.reshape(top, (nbin, nbin))
    max_top = np.max(top)
    top = top * (255/max_top)

    i=i+1
    side = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    side = np.reshape(side, (nbin, nbin))
    max_side = np.max(side)
    side = side * (255/max_side)

    i=i+1
    front = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    front = np.reshape(front, (nbin, nbin))
    max_front = np.max(front)
    front = front * (255/max_front)

    # make three independet images using good descriptor
    ### top image

    resized_img_top = cv2.resize(top, (image_size, image_size), interpolation = cv2.INTER_AREA)
    cv2.imwrite('/tmp/top.jpg', resized_img_top * contrast)
    resized_img_top = cv2.imread('/tmp/top.jpg', cv2.IMREAD_GRAYSCALE)
    resized_img_top = resized_img_top * 1. / 255
    resized_img_top = np.expand_dims(resized_img_top, axis=-1)

    ### side image
    resized_img_side = cv2.resize(side, (image_size, image_size), interpolation = cv2.INTER_AREA)
    cv2.imwrite('/tmp/side.jpg', resized_img_side * contrast)
    resized_img_side = cv2.imread('/tmp/side.jpg', cv2.IMREAD_GRAYSCALE)
    resized_img_side = resized_img_side * 1. / 255
    resized_img_side = np.expand_dims(resized_img_side, axis=-1)

    ### front image
    resized_img_front = cv2.resize(front, (image_size, image_size), interpolation = cv2.INTER_AREA)
    cv2.imwrite('/tmp/front.jpg', resized_img_front * contrast)
    resized_img_front = cv2.imread('/tmp/front.jpg', cv2.IMREAD_GRAYSCALE)
    resized_img_front = resized_img_front * 1. / 255
    resized_img_front = np.expand_dims(resized_img_front, axis=-1)

    print ("shape of input orthographic images = " +  str (resized_img_front[None].shape))


    with graph.as_default():
        with session.as_default():
            top_view_representation = encode_given_image(resized_img_top, encoder)
            side_view_representation = encode_given_image(resized_img_side, encoder)
            front_view_representation = encode_given_image(resized_img_front, encoder)
            # visualize(resized_img_top, encoder, decoder)

            print ("shape of encode representation = " + str(front_view_representation.shape))

    toc = time.clock()
    print ("deep learning took " + str (toc - tic))

    ### multi view pooling
    tic = time.clock()
    if (max_pooling) :
        global_object_representation = np.max([top_view_representation, side_view_representation, front_view_representation], axis=0)
    else:
        global_object_representation = np.average([top_view_representation, side_view_representation, front_view_representation], axis=0)
    toc = time.clock()
    #print ("pooling took " + str (toc - tic))

    print ("size of representation is "+ str(global_object_representation.shape))

    toc_global = time.clock()
    print ("########### Deep learning took " + str (time.clock() - tic_global))

    return deep_representationResponse(global_object_representation[0])

def autoencoder_server():
    rospy.init_node('autoencoder_based_representation_server')
    s = rospy.Service('autoencoder_service', deep_representation, handle_good_representation)
    print "Ready to representas based on a simple autoencoder network."
    rospy.spin()

if __name__ == "__main__":
    autoencoder_server()


