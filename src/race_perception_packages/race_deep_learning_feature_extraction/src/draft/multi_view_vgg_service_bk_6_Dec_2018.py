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
from keras.applications import vgg16
from keras.applications.vgg16 import VGG16
from keras.preprocessing import image
from keras.applications.vgg16 import preprocess_input
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
        ### create the network model
        vgg_model = vgg16.VGG16(weights='imagenet', include_top=True)
        vgg_feature_extraction = Model(inputs=vgg_model.input, outputs=vgg_model.get_layer('fc1').output)
        vgg_model._make_predict_function()
        plot_model(vgg_model, to_file='model.png')
        print(vgg_model.summary())




#__________________________________
#|                                 |
#|           GLOBAL VARS           |
#|_________________________________|

_name = 'deep_learning_services'
def handle_good_representation(req):
    
    #print (len(req.good_representation)/3)    
    nbin= int (math.sqrt(len(req.good_representation)/3))
    print ("number of bin = " + str(nbin))
    
    new_image = req.good_representation
    tic = time.clock()
    tic_global = time.clock()

    i=0
    b = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    b = np.reshape(b, (nbin, nbin))
    # print ('b = '+ str (b))
    # print ('\n\n\nsum b = ' + str(np.sum(b)))
    # print ('\n\n\nmax b = ' + str(np.max(b)))
    # print ('\n\n\nmin b = ' + str(np.min(b)))
    max_b = np.max(b)
    b = b * (255/max_b)
    # print ('\n\n\nb * 255 = ' + str(b))

    i=i+1
    g = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    g = np.reshape(g, (nbin, nbin))
    max_g = np.max(g)
    g = g * (255/max_g)
    
    #g = g * 255
    #print (g)
    
    i=i+1
    r = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    r= np.reshape(r, (nbin, nbin))
    max_r = np.max(r)
    r = r * (255/max_r)
    #r= r * 255
    #print (r)
    
    zero_image = np.zeros(nbin*nbin)
    zerolayer = np.reshape(zero_image, (nbin, nbin))

    img_b = cv2.merge((b,zerolayer,zerolayer))
    resized_img_b = cv2.resize(img_b, (224, 224))
    x_b = image.img_to_array(resized_img_b)
    x_b = np.expand_dims(x_b, axis=0)
    x_b = preprocess_input(x_b)

    img_g = cv2.merge((g,zerolayer,zerolayer))
    resized_img_g = cv2.resize(img_g, (224, 224))
    x_g = image.img_to_array(resized_img_g)
    x_g = np.expand_dims(x_g, axis=0)
    x_g = preprocess_input(x_g)


    img_r = cv2.merge((r,zerolayer,zerolayer))
    resized_img_r = cv2.resize(img_r, (224, 224))
    x_r = image.img_to_array(resized_img_r)
    x_r = np.expand_dims(x_r, axis=0)
    x_r = preprocess_input(x_r)
    toc = time.clock()

    print ("image preparation took " + str (toc - tic))

    

    #CV_32SC3


    img_good = cv2.merge((b,g,r))
    resized_image_tmp = cv2.resize(img_good, (224, 224), interpolation = cv2.INTER_AREA)
    #resized_image = resized_image_tmp.astype(np.float32, copy=False)
    resized_image = resized_image_tmp
    #cv2.imshow ('test',resized_image_tmp)
    cv2.imwrite('/tmp/6.jpg', resized_image)


    #normalization 
    #The mean pixel values are taken from the VGG authors, which are the values computed from the training dataset.
    mean_pixel = [103.939, 116.779, 123.68]
    for c in range(3):
        resized_image[:, :, c] = resized_image[:, :, c] - mean_pixel[c]
    
    #img_good = img_good.transpose((2,0,1))
    #img_good = np.expand_dims(img_good, axis=0)

    #cv2.imshow ('normalized', resized_image)
    #cv2.waitKey(0)

    
    #img_good =cv2.merge((b,b,b))
    #img_good = cv2.imread("/home/hamidreza/Pictures/initial.jpg")
    #img_good = cv2.imread("/home/hamidreza/Pictures/puppy-dog.jpg")
    x = image.img_to_array(resized_image)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)
    tic = time.clock()

    # plt.imshow (img_good)
    #plt.imshow (img_good[:,:,2])
    # plt.show()

    with graph.as_default():  
        with session.as_default():    
            fc2_features = vgg_feature_extraction.predict(x)
            ### multi view
            # fc2_features_b = vgg_feature_extraction.predict(x_b)
            # fc2_features_g = vgg_feature_extraction.predict(x_g)
            # fc2_features_r = vgg_feature_extraction.predict(x_r)

    toc = time.clock()
    print ("deep learning took " + str (toc - tic))
    #deep_representation_b = fc2_features_b.reshape((4096,1))
    #deep_representation_g = fc2_features_g.reshape((4096,1))
    #deep_representation_r = fc2_features_r.reshape((4096,1))

    ### multi view
    # tic = time.clock()
    # fc2_features = np.max([fc2_features_b, fc2_features_g, fc2_features_r], axis=0)
    # toc = time.clock()
    # print ("pooling took " + str (toc - tic))
    
    print ("size of representation is "+ str(len(fc2_features[0])))

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
    
    return vgg16_modelResponse(fc2_features[0])

def vgg16_server():
    rospy.init_node('deep_learning_representation_server')
    s = rospy.Service('vgg_service', vgg16_model, handle_good_representation)
    print "Ready to representas based on vgg16 network."
    rospy.spin()

if __name__ == "__main__":
    vgg16_server()


