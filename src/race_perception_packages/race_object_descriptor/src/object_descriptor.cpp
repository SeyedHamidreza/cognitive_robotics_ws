#ifndef _OBJECT_DESCRIPTOR_CPP_
#define _OBJECT_DESCRIPTOR_CPP_

#include <object_descriptor/object_descriptor.h>
using namespace race_object_descriptor;

//Can only be one template class if this is a lib for use as a nodelet
template class ObjectDescriptor<pcl::PointXYZRGBA>;

#endif
