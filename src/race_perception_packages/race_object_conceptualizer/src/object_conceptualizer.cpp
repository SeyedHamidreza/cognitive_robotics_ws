#ifndef _OBJECT_CONCEPTUALIZER_CPP_
#define _OBJECT_CONCEPTUALIZER_CPP_

#include <object_conceptualizer/object_conceptualizer.h>
using namespace race_object_conceptualizer;

//Can only be one template class if this is a lib for use as a nodelet
template class ObjectConceptualizer<pcl::PointXYZRGBA>;

#endif
