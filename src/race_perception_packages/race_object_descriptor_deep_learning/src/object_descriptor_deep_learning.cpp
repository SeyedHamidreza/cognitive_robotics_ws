#ifndef _OBJECT_DESCRIPTOR_DEEP_LEARNING_CPP_
#define _OBJECT_DESCRIPTOR_DEEP_LEARNING_CPP_

#include <object_descriptor_deep_learning/object_descriptor_deep_learning.h>
using namespace race_object_descriptor_deep_learning;

//Can only be one template class if this is a lib for use as a nodelet
template class ObjectDescriptorDeepLearning<pcl::PointXYZRGBA>;

#endif
