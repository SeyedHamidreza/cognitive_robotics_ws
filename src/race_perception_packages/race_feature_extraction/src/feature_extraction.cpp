#ifndef _FEATURE_EXTRACTION_CPP_
#define _FEATURE_EXTRACTION_CPP_

#include <feature_extraction/feature_extraction.h>
using namespace race_feature_extraction;

//Can only be one template class if this is a lib for use as a nodelet
template class FeatureExtraction<pcl::PointXYZRGBA>;

#endif
