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

from pytictoc import TicToc


from keras.models import load_model
from tensorflow import Graph, Session

# this is necessary for multi threading and server - client system
graph = tf.Graph()
with graph.as_default():
    session = tf.Session()
    with session.as_default():    
        ### create the network model
        vgg_model = vgg16.VGG16(weights='imagenet', include_top=True)
        vgg_feature_extraction = Model(inputs=vgg_model.input, outputs=vgg_model.get_layer('fc2').output)
        vgg_model._make_predict_function()



#__________________________________
#|                                 |
#|           GLOBAL VARS           |
#|_________________________________|

_name = 'deep_learning_services'

def handle_good_representation(req):
    
    print (len(req.good_representation)/3)    
    nbin= int (math.sqrt(len(req.good_representation)/3))
    print nbin
    
    new_image = req.good_representation
    
    i=0
    b = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    b = np.reshape(b, (nbin, nbin))
    b = b * 255
    #print (b)

    i=i+1
    #g = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    g= np.zeros(nbin*nbin)
    g = np.reshape(g, (nbin, nbin))
    g = g * 255
    #print (g)
    
    #i=i+1
    #r = new_image[i*nbin*nbin:(i+1)*nbin*nbin]
    r= np.zeros(nbin*nbin)
    r= np.reshape(r, (nbin, nbin))
    r= r * 255
    #print (r)
    
    img_good = cv2.merge((b,g,r))
    
    #plt.imshow (img_good)
    #plt.imshow (img_good[:,:,2])
    #plt.show()
    
    resized_image = cv2.resize(img_good, (224, 224))
    x = image.img_to_array(resized_image)
    x = np.expand_dims(x, axis=0)
    x = preprocess_input(x)

    with graph.as_default():  
        with session.as_default():    
            fc2_features = vgg_feature_extraction.predict(x)
        
        
    deep_representation = fc2_features.reshape((4096,1))
    
    #print len(fc2_features[0])
    return vgg16_modelResponse(fc2_features[0])

def vgg16_server():
    rospy.init_node('deep_learning_representation_server')
    s = rospy.Service('vgg_service', vgg16_model, handle_good_representation)
    print "Ready to representas based on vgg16 network."
    rospy.spin()

if __name__ == "__main__":
    vgg16_server()


