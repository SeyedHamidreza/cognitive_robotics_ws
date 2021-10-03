#!/usr/bin/env python

#__________________________________
#|                                 |
#|           IMPORTS               |
#|_________________________________|

### general
import roslib; #roslib.load_manifest('rug_deep_feature_extraction')
import rospy
from rug_deep_feature_extraction.srv import *
from nodelet.srv import *
import random
import os
import sys
import threading
import subprocess
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")

from cv_bridge import CvBridge, CvBridgeError

import tensorflow as tf
from tensorflow import keras
from keras.applications import *
#from keras.applications.mobilenet import MobileNet
from keras.preprocessing import image
from keras.applications.mobilenet import preprocess_input
from keras.models import Model
from tensorflow.keras import layers
from keras import backend as K
from keras.utils import plot_model

tf.config.optimizer.set_jit(True) # Reduces latency by using XLA features

from pytictoc import TicToc
import time

# this is needed to get rid of cpu warning AUX
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

from tensorflow import Graph, Session
from tensorflow.keras.models import load_model

np.set_printoptions(threshold=sys.maxsize)
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


# "/vgg16_service", "/vgg19_service", "/xception_service", "/resnet50_service", "/mobilenet_service",  "/mobilenetV2_service", 
# "/densenet121_server", "/densenet169_server", "/densenet201_server", "/nasnet_large_server", "/nasnet_mobile_server", 
# "/inception_resnet_service", "/inception_service", "/autoencoder_service" 

base_network = str(sys.argv[1])
# print ("1 --- base_network = " + str(base_network))


recognition_network = "MobileNet"
image_size = 224 # Makes this a global parameter instead

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
        if (base_network == "vgg16_fc1"):
            vgg_model = vgg16.VGG16(weights='imagenet', include_top=True)
            encoder = Model(inputs=vgg_model.input, outputs=vgg_model.get_layer('fc1').output)
            #vgg_model._make_predict_function()
            #plot_model(vgg_model, to_file='model.png')
            print(vgg_model.summary())

        elif (base_network == "vgg16_fc2"):
            vgg_model = vgg16.VGG16(weights='imagenet', include_top=True)
            encoder = Model(inputs=vgg_model.input, outputs=vgg_model.get_layer('fc2').output)
            #vgg_model._make_predict_function()
            #plot_model(vgg_model, to_file='model.png')
            print(vgg_model.summary())

        elif (base_network == "vgg19_fc1"):
            vgg_model = vgg19.VGG19(weights='imagenet', include_top=True)
            encoder = Model(inputs=vgg_model.input, outputs=vgg_model.get_layer('fc1').output)
            #vgg_model._make_predict_function()
            #plot_model(vgg_model, to_file='model.png')
            print(vgg_model.summary())

        elif (base_network == "vgg19_fc2"):
            vgg_model = vgg19.VGG19(weights='imagenet', include_top=True)
            encoder = Model(inputs=vgg_model.input, outputs=vgg_model.get_layer('fc2').output)
            #vgg_model._make_predict_function()
            #plot_model(vgg_model, to_file='model.png')
            print(vgg_model.summary())

        elif (base_network == "xception"):
            xception_model = xception.Xception(weights='imagenet', include_top=True)
            encoder = Model(inputs=xception_model.input, outputs=xception_model.get_layer('avg_pool').output)
            image_size = 299 #fix
            #xception_model._make_predict_function()
            #plot_model(xception_model, to_file='model.png')
            print(xception_model.summary())

        elif (base_network == "resnet50"):
            resnet_model = resnet50.ResNet50(weights='imagenet', include_top=True)
            encoder = Model(inputs=resnet_model.input, outputs=resnet_model.get_layer('avg_pool').output)
            #resnet_model._make_predict_function()
            #plot_model(resnet_model, to_file='model.png')
            print(resnet_model.summary())

        elif (base_network == "mobileNet"):
            mobilenet_model = mobilenet.MobileNet(weights='imagenet', include_top=True)
            encoder = Model(inputs=mobilenet_model.input, outputs=mobilenet_model.get_layer('global_average_pooling2d_1').output)
            #mobilenet_model._make_predict_function()
            #plot_model(mobilenet_model, to_file='model.png')
            print(mobilenet_model.summary())

        elif (base_network == "mobileNetV2"):
            mobilenet_model = mobilenet_v2.MobileNetV2(weights='imagenet', include_top=True)
            encoder = Model(inputs=mobilenet_model.input, outputs=mobilenet_model.get_layer('global_average_pooling2d_1').output)
            #mobilenet_model._make_predict_function()
            #plot_model(mobilenet_model, to_file='model.png')
            print(mobilenet_model.summary())

        elif (base_network == "denseNet121"):
            densenet121_model = densenet.DenseNet121(weights='imagenet', include_top=True)
            encoder = Model(inputs=densenet121_model.input, outputs=densenet121_model.get_layer('avg_pool').output)
            #densenet121_model._make_predict_function()
            #plot_model(densenet121_model_model, to_file='densenet121.png')
            print(densenet121_model.summary())

        elif (base_network == "denseNet169"):
            densenet169_model = densenet.DenseNet169(weights='imagenet', include_top=True)
            encoder = Model(inputs=densenet169_model.input, outputs=densenet169_model.get_layer('avg_pool').output)
            #densenet169_model._make_predict_function()
            #plot_model(densenet169_model_model, to_file='densenet169.png')
            print(densenet169_model.summary())

        elif (base_network == "denseNet201"):
            densenet201_model = densenet.DenseNet201(weights='imagenet', include_top=True)
            encoder = Model(inputs=densenet201_model.input, outputs=densenet201_model.get_layer('avg_pool').output)
            #densenet201_model._make_predict_function()
            #plot_model(densenet201_model_model, to_file='densenet201.png')
            print(densenet201_model.summary())
            
        elif (base_network == "nasnetLarge"):
            nasnet_large_model = nasnet.NASNetLarge(weights='imagenet', include_top=True)
            encoder = Model(inputs=nasnet_large_model.input, outputs=nasnet_large_model.get_layer('global_average_pooling2d_1').output)
            #nasnet_large_model._make_predict_function()
            #plot_model(nasnet_large_model, to_file='nasnet_large.png')
            print(nasnet_large_model.summary())
            
        elif (base_network == "nasnetMobile"):
            nasnet_mobile_model = nasnet.NASNetMobile(weights='imagenet', include_top=True)
            encoder = Model(inputs=nasnet_mobile_model.input, outputs=nasnet_mobile_model.get_layer('global_average_pooling2d_1').output)
            #nasnet_mobile_model._make_predict_function()
            # plot_model(nasnet_mobile_model, to_file='nasnet_mobile.png')
            print(nasnet_mobile_model.summary())
            
        elif (base_network == "inception"):
            inception_model = inception_v3.InceptionV3(weights='imagenet', include_top=True)
            encoder = Model(inputs=inception_model.input, outputs=inception_model.get_layer('avg_pool').output)
            image_size = 299 # fix
            #inception_model._make_predict_function()
            #plot_model(inception_model, to_file='model.png')
            print(inception_model.summary())

        elif (base_network == "inceptionResnet"):
            inception_resnet_model = inception_resnet_v2.InceptionResNetV2(weights='imagenet', include_top=True)
            encoder = Model(inputs=inception_resnet_model.input, outputs=inception_resnet_model.get_layer('avg_pool').output)
            #inception_resnet_model._make_predict_function()
            #plot_model(inception_resnet_model, to_file='model.png')
            print(inception_resnet_model.summary())     

        elif (base_network == "effNet"):
            effnet_model = efficientnet.EfficientNetB3(weights='imagenet', include_top=True)
            encoder = Model(inputs=effnet_model.input, outputs=effnet_model.get_layer('avg_pool').output)
            #inception_resnet_model._make_predict_function()
            #plot_model(inception_resnet_model, to_file='model.png')
            print(effnet_model.summary())     
        else:
            print("The selected network has not been implemented yet -- please choose another network!")
            exit() 

tmp_img = 0

def handle_deep_representation(req):
    global image_size # allows to fix the global parameter for the image

    #__________________________
    #|                         |
    #|     ROS PARAMETERS      |
    #|_________________________|

    ## RULE: 0 : FALSE, 1 : TRUE
    image_normalization = 0 #FALSE 
    multiviews = 1 #TRUE 
    pooling_function = "MAX" #TRUE 
    number_of_bins = 150
    gui = True
    if rospy.has_param('/perception/image_normalization'):
        image_normalization = rospy.get_param("/perception/image_normalization")
        #print ("########## image_normalization (0 : FALSE, 1 : TRUE) = " + str(image_normalization))

    if rospy.has_param('/perception/multiviews'):
        multiviews = rospy.get_param("/perception/multiviews")
        #print ("########## multiviews (0 : FALSE, 1 : TRUE) = " + str(multiviews))

    if rospy.has_param('/perception/base_network'):
        base_network = rospy.get_param("/perception/base_network")
        # print ("\t - base_network = " + str(base_network))

    if rospy.has_param('/perception/pooling_function'):
        pooling_function = rospy.get_param("/perception/pooling_function")
        # print ("\t - pooling_function = " + str(pooling_function))
    
    if rospy.has_param('/perception/orthographic_image_resolution'):
        number_of_bins = rospy.get_param("/perception/orthographic_image_resolution")
        # print ("\t - orthographic_image_resolution = " + str(number_of_bins)) 

    if rospy.has_param('/perception/gui'):
        gui = rospy.get_param("/perception/gui")
        # print ("\t - gui = " + str(gui)) 

    #__________________________
    #|                         |
    #|    MAIN SERVICE CODE    |
    #|_________________________|

    number_of_views = int(len(req.good_representation) / (number_of_bins*number_of_bins))
    # print ("\t - number_of_views = " + str(number_of_views))
                
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
                resized_img, othographic_image = preprocessingForOrthographicImages(img, image_size)

                img_g = cv2.merge((othographic_image, othographic_image, othographic_image))
                x_r = image.img_to_array(img_g)
                x_r = np.expand_dims(x_r, axis=0)
                x_r = preprocess_input(x_r)
                feature = encoder.predict(x_r)          
                
        # pooling functions
        if (i == 0):
            global_object_representation = feature            
        elif (pooling_function == "MAX"):
            global_object_representation = np.max([global_object_representation, feature], axis=0)
        elif (pooling_function == "AVG"):
            global_object_representation = np.average([global_object_representation, feature], axis=0)
        elif (pooling_function == "APP"):
            global_object_representation = np.append(global_object_representation, feature, axis=1)


    ### deep feature vector of rgb and depth imgaes
    try:

        bridge = CvBridge()
        cv_rgb_image = bridge.imgmsg_to_cv2(req.RGB_image, "bgr8")
        resized_rgb_img, othographic_image = preprocessingForOrthographicImages(cv_rgb_image, image_size)                            

        if (gui):
            cv2.imshow('RGB_image', resized_rgb_img)
            cv2.waitKey(1)

        cv_depth_image = bridge.imgmsg_to_cv2(req.depth_image, "bgr8")      
        resized_depth_img, othographic_depth_image = preprocessingForOrthographicImages(cv_depth_image, image_size)                
    
        if (gui):
            cv2.imshow('Depth_image', resized_depth_img)
            cv2.waitKey(1)

        #### encode RGB image
        with graph.as_default():
            with session.as_default():    
                x_rgb = image.img_to_array(resized_rgb_img)
                x_rgb = np.expand_dims(x_rgb, axis=0)
                x_rgb = preprocess_input(x_rgb)
                feature = encoder.predict(x_rgb)
            
        # pooling functions # size of feature can be check first and then do this part
        if (pooling_function == "MAX"):
            global_object_representation = np.max([global_object_representation, feature], axis=0)
        elif (pooling_function == "AVG"):
            global_object_representation = np.average([global_object_representation, feature], axis=0)
        elif (pooling_function == "APP"):
            global_object_representation = np.append(global_object_representation, feature, axis=1)

        #### encode Depth image
        with graph.as_default():
            with session.as_default():          
                x_depth = image.img_to_array(resized_depth_img)
                x_depth = np.expand_dims(x_depth, axis=0)
                x_depth = preprocess_input(x_depth)
                feature = encoder.predict(x_depth)
            
        # pooling functions # size of feature can be check first and then do this part
        if (pooling_function == "MAX"):
            global_object_representation = np.max([global_object_representation, feature], axis=0)
        elif (pooling_function == "AVG"):
            global_object_representation = np.average([global_object_representation, feature], axis=0)
        elif (pooling_function == "APP"):
            global_object_representation = np.append(global_object_representation, feature, axis=1)

    except CvBridgeError as e:
        print(e)
        print ("error visualize image")
    
    
    toc_global = time.clock()
    # print ("\t - size of representation is "+ str(global_object_representation.shape))
    # print ("\t - deep object representation took " + str (toc_global - tic_global))
    # print ("----------------------------------------------------------")            
    return deep_representationResponse(global_object_representation[0])


def RGBD_multiview_service():
    rospy.init_node('deep_learning_representation_server')
    s = rospy.Service('RGBD_multiview_service', deep_representation, handle_deep_representation)
    print("Ready to representas RGBD object based on " + base_network + " network.")
    rospy.spin()

if __name__ == "__main__":
    RGBD_multiview_service()
