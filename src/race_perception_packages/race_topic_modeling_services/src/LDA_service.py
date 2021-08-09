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
from gensim.models import HdpModel, LdaModel, CoherenceModel
from gensim import corpora


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



""" 
--------------------------------------------------------------------
Manhattan distance 
--------------------------------------------------------------------
"""
def manhattanDistance(firstList,secondList):
    dist = sum(abs(a-b) for a,b in zip(firstList,secondList))
    return dist

""" 
--------------------------------------------------------------------
Convert the predicted topic to a vector with V(number of words in vocabulary) length 
--------------------------------------------------------------------
"""
def topic_to_vector(topic):
    topic_vector = [0]*number_of_topics
    topic_probs = [prob for (x,prob) in topic]
    topic_indices= [x for (x,prob) in topic]
    for i in range(0,number_of_topics):
        for j in range(0,len(topic_indices)):
            if i==topic_indices[j]:
                topic_vector[i]=topic_probs[j]

    return topic_vector


""" 
--------------------------------------------------------------------
Compare the predicted topics for test data with all the topics predicted for all the training data 
--------------------------------------------------------------------
"""
def compare_topic_histograms(topic,learnedTopics,category):
    min_distance=100000

    for learnedTopic in learnedTopics[category]:
        topic_vector=topic_to_vector(topic)
        learned_topic_vector=topic_to_vector(learnedTopic)
        dist = manhattanDistance(topic_vector,learned_topic_vector)
        if(dist<min_distance):
            min_distance = dist

    return min_distance

""" 
--------------------------------------------------------------------
Delete empty lines form file
--------------------------------------------------------------------
"""
def deleteEmptyLines(text):
    output=[]
    for line in text:
        if line != '':
            output.append(line)
    return output

""" 
--------------------------------------------------------------------
Is the prediction true for the current BOW?
--------------------------------------------------------------------
"""
def predictionIsTrue(ldas,BoW,true_category,learnedCats):
    likelihoods=[]
    # for i in range(len(categories)):
    #     likelihoods.append(ldas[categories[i]].evaluate_test_corpus([BoW]))
    for lda in ldas:
        likelihoods.append(lda.log_perplexity([BoW]))
    index = likelihoods.index(max(likelihoods))
    if true_category == learnedCats[index]:
        return True,learnedCats[index]
    return False,learnedCats[index]


def initializeHDPModel(max_number_of_topics_for_lda,num_vocab):
    print("Begin modeling with {} words in Vocabulary.".format(str(num_vocab)))
    number_of_words_in_dictionary = int(num_vocab)
    """ 
    --------------------------------------------------------------------
    Making dictionary for HDP (In this implementation since the BoW is constructed using Categories 
    there is no need for constructing real dictionary and the range (0, number_of_words_in_dictionary) is enough
    --------------------------------------------------------------------
    """
    dict_words = np.arange(0, number_of_words_in_dictionary)
    dict_words = [[str(i) for i in dict_words]]
    dictionary = corpora.Dictionary(dict_words)

    return dictionary

def testingPhase_HDP(ground_truth_label,BoW_representation):
    """
    --------------------------------------------------------------------
    Testing phase
    --------------------------------------------------------------------
    """
    BoW = []
    cc = 0
    for b in BoW_representation:
        temp = (cc, b)
        BoW.append(temp)
        cc += 1

    # predictStatus, predictionCategory = predictionIsTrue(ldas, BoW, ground_truth_label, learned_categories)
    topics=lda[BoW]
    print(topics)

    distances=[]
    for cat in range(0, len(learned_categories)):
        distances.append(compare_topic_histograms(topics, predicted_topics_per_category, cat))
    index = distances.index(min(distances))
    predicted_topics_per_category[learned_categories.index(ground_truth_label)].append(topics)
    print(learned_categories)
    print(distances)
    predictionCategory = learned_categories[index]

    # print("Prediction is: ", predictStatus)
    print("Predicted Label: {}, True Category: {} ".format(predictionCategory,ground_truth_label))

    return predictionCategory

def train_the_HDP_models(ground_truth_label,BoW_representation):
    """
    --------------------------------------------------------------------
    Traning phase
    --------------------------------------------------------------------
    """
    BoW = []
    cc = 0
    for b in BoW_representation:
        temp = (cc, b)
        BoW.append(temp)
        cc += 1
    BoWs.append(BoW)
    for i in range(10):
        lda.update(BoWs)
    return 0

def add_new_category(ground_truth_label):
    learned_categories.append(ground_truth_label)
    predicted_topics_per_category.append([])
    # ldas.append(HdpModel(corpus=None, id2word=dictionary, chunksize=1, K=10,
    #                      T=max_number_of_topics_for_lda))  # note that in gensim self.m_D is set to 1 by us
    # ldas.append(LdaModel(corpus=None, id2word=dictionary, chunksize=1,num_topics=30))
    # note that in gensim slef.m_D is set to 1 by us
    # BoWs.append([])
    return 0

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
    # print (BoW_representation)
    ground_truth_label =  req.ground_truth_label
    # print ("ground_truth_label = " + ground_truth_label)
    # print("True Category:", ground_truth_label)
    # teach or test data
    teach_flag = req.teach_data
    # print("teach data = " + str(teach_flag))


    ### your code should be added here
    predictionCategory = "empty"
    if (teach_flag):
        print("**************************************** TEACH {}************************************"
              "**********".format(len(learned_categories)))
        if ground_truth_label not in learned_categories:
            add_new_category(ground_truth_label)

        train_the_HDP_models(ground_truth_label,BoW_representation)
        print("model is trained")
    else:
        print("=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+= TEST =+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+")
        predictionCategory = testingPhase_HDP(ground_truth_label,BoW_representation)

    topic_info = lda.print_topics(num_topics=1, num_words=90)
    # print(topic_info)
    ### making a response message first and then fill its fields; finally return it
    result = topic_modellingResponse()
    result.topic_representation = (1,2,2,1,1,1)
    result.recognition_result = predictionCategory
	
    return result

def local_HDP_server():

    rospy.init_node('topic_modelling_local_HDP_server')
    s = rospy.Service('local_HDP_service', topic_modelling, handle_HDP_representation_and_recognition)
    # This declares a new service named local_HDP_service with the topic_modelling service type. All
    # requests are passed to handle_HDP_representation_and_recognition function.
    # handle_HDP_representation_and_recognition is called with instances of AddTwoIntsRequest and
    # returns instances of AddTwoIntsResponse.
    print( "*** Ready to represent and recognize a given 3D object based on Local-HDP algorithm ***")
    rospy.spin()

# if __name__ == "__main__":

number_of_topics = 100
dictionary_size = 90
num_learned_categories=0
learned_categories = []
BoWs=[]
predicted_topics_per_category=[]
dictionary = initializeHDPModel(number_of_topics, dictionary_size)
lda=LdaModel(corpus=None, id2word=dictionary, chunksize=1,num_topics=number_of_topics)
local_HDP_server()

