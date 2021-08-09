#ifndef _OBJECT_CONCEPTUALIZER_LIB_H_
#define _OBJECT_CONCEPTUALIZER_LIB_H_


/* _________________________________
  |                                 |
  |           Defines               |
  |_________________________________| */
#ifndef _OBJECT_CONCEPTUALIZER_LIB_DEBUG_
#define _OBJECT_CONCEPTUALIZER_LIB_DEBUG_ TRUE
#endif

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//Boost includes
#include <boost/make_shared.hpp>
//ROS includes
#include <ros/ros.h>

//PCL includes
#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>

//Eigen includes
#include <Eigen/Core>

//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <ctime>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>    
#include <string.h>
#include <cctype> // string manupolation
#include <math.h> 
#include <cstring>


//package includes
#include <race_perception_msgs/perception_msgs.h>

//Gi includes
#include <pluginlib/class_list_macros.h> 
#include <stdio.h>
#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>
#include <race_perception_db/MsgTest.h>
#include <race_perception_utils/cycle.h>
#include <race_perception_utils/print.h>


/* _________________________________
   |                                 |
   |           NAMESPACES            |
   |_________________________________| */

using namespace pcl;
using namespace race_perception_msgs;
using namespace race_perception_db;
using namespace race_perception_utils;

typedef PointXYZRGBA PointT; // define new type.


/* _________________________________
   |                                 |
   |        FUNCTION PROTOTYPES      |
   |_________________________________| */
int KLdiffrenceBetweenTwoObjectViewHistogram(SITOV objectViewHistogram1,
					      SITOV objectViewHistogram2, 
					      double &diffrence);

int histogramBasedObjectCategoryKLDistance( SITOV target,
		vector< SITOV > category_instances,
		float &minimumDistance, 
		int &best_matched_index, 
		PrettyPrint &pp);
/**
 * @brief intraCategoryDistance: compute Intra Category Distance
 * @param category_instances : vector of objectview spinimages (Input)
 * @param ICD : return IntraCategoryDistance to ICD parameter (Output)
 * @return 
 */	
int intraCategoryDistance(vector< vector <SITOV> > category_instances, double &ICD, PrettyPrint &pp);

/**
 * @brief objectCategoryDistance: compute Object Category Distance 
 * @param target: vector of SITOV that reperesent the spinimages of target object (Input)
 * @param category_instances: represent vector of objectview spinimges of particular category (Input)
 * @param minimumDistance: return objectCategoryDistance to minimumDistance parameter (Output)
 * @param best_match_index: return best matched view index (Output)
 * @return 
 */
int objectCategoryDistance( vector <SITOV> target,
			    vector< vector <SITOV> > category_instances,
			    float &minimumDistance, int &best_matched_index, PrettyPrint &pp);

int KLobjectCategoryDistance( vector <SITOV> target,
		vector< vector <SITOV> > category_instances,
		float &minimumDistance, int &best_matched_index , PrettyPrint &pp);

int averageObjectCategoryDistance( vector <SITOV> target,
		vector< vector <SITOV> > category_instances,
		float &average_distance,  PrettyPrint &pp);

/**
 * @brief normalizedObjectCategoryDistance: normalize Object Category Distance for particular category 
 * @param minimumDistance: minimum object category distance to particular category(Input)
 * @param ICD: represent Intra category distance for particular category (Input)
 * @param normalizedDistance: return normalizedObjectCategoryDistance to normalizedDistance parameter (Output)
 * @return 
 */
int normalizedObjectCategoryDistance( float minimumDistance, float ICD , float &normalizedDistance, PrettyPrint &pp);



int computingConfidence(vector <NOCD> & normalizedObjectCategoriesDistance, PrettyPrint &pp);




int simpleClassificationRule( vector<float> normalizedObjectCategoriesDistances ,
			int &categoryIndex, float &confidenceValue, 
			float recognitionThreshold, float &minimum_distance, PrettyPrint &pp );
/**
 * @brief classificationRule: normalize Object Category Distance for particular category 
 * @param minimumDistance: minimum object category distance to particular category(Input)
 * @param ICD: represent Intra category distance for particular category (Input)
 * @param normalizedDistance: return normalizedObjectCategoryDistance to normalizedDistance parameter (Output)
 * @param recognitionThreshold: recognition threshold for classification Rule
 * @return 
 */
int classificationRule( vector<float> normalizedObjectCategoriesDistances, 
                        int &categoryIndex, float &confidenceValue, float recognitionThreshold, 
			float &minimum_distance, PrettyPrint &pp );

int newClassificationRule( vector <NOCD> normalizedObjectCategoriesDistance,
		int &categoryIndex, 
		float recognitionThreshold, float &minimum_distance, PrettyPrint &pp );


int putObjectViewSpinImagesinSpecificCategory (std::string cat_name, unsigned int cat_id, 
					    unsigned int track_id, unsigned int view_id, 
					    vector <SITOV> SpinImageMsg, PrettyPrint &pp
				     );

int diffrenceBetweenTwoObjectViewHistogram(SITOV objectViewHistogram1,
					   SITOV objectViewHistogram2, 
					   float &diffrence);


int histogramBasedObjectCategoryDistance( SITOV target,
					vector< SITOV > category_instances,
					float &minimumDistance, 
					int &best_matched_index, 
					PrettyPrint &pp);



int updateNaiveBayesModel1(std::string cat_name,
			  unsigned int cat_id,
			  PrettyPrint &pp,
			  PerceptionDB* _pdb1
  			);



#endif

