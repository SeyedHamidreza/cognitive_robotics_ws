#!/usr/bin/env python

# __________________________________
# |                                 |
# |           IMPORTS               |
# |_________________________________|

import roslib;

roslib.load_manifest('race_topic_modeling_services')
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
Delete empty lines form file
--------------------------------------------------------------------
"""


def deleteEmptyLines(text):
    output = []
    for line in text:
        if line != '':
            output.append(line)
    return output


""" 
--------------------------------------------------------------------
Is the prediction true for the current BOW?
--------------------------------------------------------------------
"""


def predictionIsTrue(ldas, BoW, true_category, learnedCats):
    likelihoods = []
    # for i in range(len(categories)):
    #     likelihoods.append(ldas[categories[i]].evaluate_test_corpus([BoW]))
    for lda in ldas:
        likelihoods.append(lda.log_perplexity([BoW]))
    index = likelihoods.index(max(likelihoods))
    if true_category == learnedCats[index]:
        return True, learnedCats[index]

    return False, learnedCats[index]


def initializeHDPModel(number_of_topics, num_vocab):
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


def testingPhase_HDP(ground_truth_label, BoW_representation):
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

    predictStatus, predictionCategory = predictionIsTrue(ldas, BoW, ground_truth_label, learned_categories)
    print("Prediction is: ", predictStatus)
    print("Predicted Label: {}, True Category: {} ".format(predictionCategory, ground_truth_label))

    return predictionCategory


def train_the_HDP_models(ground_truth_label, BoW_representation):
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
    BoWs[learned_categories.index(ground_truth_label)].append(BoW)
    ldas[learned_categories.index(ground_truth_label)] = LdaModel(
        corpus=BoWs[learned_categories.index(ground_truth_label)]
        , id2word=dictionary, chunksize=1,
        num_topics=number_of_topics)
    for i in range(10):
        ldas[learned_categories.index(ground_truth_label)].update(BoWs[learned_categories.index(ground_truth_label)])
    return 0


def add_new_category(ground_truth_label):
    learned_categories.append(ground_truth_label)
    ldas.append(LdaModel(corpus=None, id2word=dictionary, chunksize=1, num_topics=number_of_topics))
    # note that in gensim self.m_D is set to 1 by us
    # ldas.append(LdaModel(corpus=None, id2word=dictionary, chunksize=1,num_topics=30))  # note that in gensim slef.m_D is set to 1 by us
    BoWs.append([])
    return 0


def handle_HDP_representation_and_recognition(req):
    # __________________________
    # |                         |
    # |     ROS PARAMETERS      |
    # |_________________________|

    ## NOTE: all parametres should be defined in launch files  
    ## RULE: 0 : FALSE, 1 : TRUE

    # example:
    image_normalization = 0  # FALSE

    if rospy.has_param('/perception/image_normalization'):
        image_normalization = rospy.get_param("/perception/image_normalization")
        # print ("########## image_normalization (0 : FALSE, 1 : TRUE) = " + str(image_normalization))

    # __________________________
    # |                         |
    # |    MAIN SERVICE CODE    |
    # |_________________________|

    BoW_representation = req.bag_of_words_representation
    # print (BoW_representation)
    ground_truth_label = req.ground_truth_label
    # print ("ground_truth_label = " + ground_truth_label)
    # print("True Category:", ground_truth_label)
    # teach or test data
    teach_flag = req.teach_data
    # print("teach data = " + str(teach_flag))

    ### your code should be added here
    predictionCategory = "empty"
    if (teach_flag):
        print("**************************************** TEACH {}**********************************************".format(len(learned_categories)))
        if ground_truth_label not in learned_categories:
            add_new_category(ground_truth_label)

        train_the_HDP_models(ground_truth_label, BoW_representation)
        print("model is trained")
    else:
        print("=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+= TEST =+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+")
        predictionCategory = testingPhase_HDP(ground_truth_label, BoW_representation)

    topic_info = ldas[learned_categories.index(ground_truth_label)].print_topics(num_topics=1, num_words=90)
    # print(topic_info)
    ### making a response message first and then fill its fields; finally return it
    result = topic_modellingResponse()
    result.topic_representation = (1, 2, 2, 1, 1, 1)
    result.recognition_result = predictionCategory

    return result


def local_HDP_server():
    rospy.init_node('topic_modelling_local_HDP_server')
    s = rospy.Service('local_HDP_service', topic_modelling, handle_HDP_representation_and_recognition)
    # This declares a new service named local_HDP_service with the topic_modelling service type. All requests are passed to handle_HDP_representation_and_recognition function. 
    # handle_HDP_representation_and_recognition is called with instances of AddTwoIntsRequest and returns instances of AddTwoIntsResponse.
    print("*** Ready to represent and recognize a given 3D object based on Local-HDP algorithm ***")
    rospy.spin()


# if __name__ == "__main__":

number_of_topics = 30
dictionary_size = 90
num_learned_categories = 0
learned_categories = []
BoWs = []
ldas = []
dictionary = initializeHDPModel(number_of_topics, dictionary_size)
local_HDP_server()
