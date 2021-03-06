#ifndef _SPIN_IMAGE_H_
#define _SPIN_IMAGE_H_


/* _________________________________
  |                                 |
  |           Defines               |
  |_________________________________| */
#ifndef _SPIN_IMAGE_DEBUG_
#define _SPIN_IMAGE_DEBUG_TRUE
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

//this package includes
// #include <race_feature_extraction/SpinImages.h>


//package includes
#include <race_perception_msgs/perception_msgs.h>

//Gi includes
#include <pluginlib/class_list_macros.h> 
#include <stdio.h>
#include <race_perception_db/perception_db_serializer.h>
#include <race_perception_db/MsgTest.h>


/* _________________________________
   |                                 |
   |           NAMESPACES            |
   |_________________________________| */

using namespace pcl;
using namespace std;
using namespace race_perception_msgs;
using namespace race_perception_db;
//using namespace pcl;

/* _________________________________
   |                                 |
   |           TYPEDEFS              |
   |_________________________________| */
   
typedef Histogram<153> SpinImage;
typedef PointXYZRGBA PointT;  
   
#define DEFINE_VARIABLE_REPRESENTATION(PointT)   template <>  \
	class DefaultPointRepresentation<PointT> : public PointRepresentation<PointT> \
{ \
	protected:\
			  static int var_nr_dimensions_;\
	\
	public:\
		   DefaultPointRepresentation ()\
	{ \
		this->trivial_ =true;\
		this->nr_dimensions_ = var_nr_dimensions_;\
	}\
	static void setNumberofDimensions(const PointT &p){\
		var_nr_dimensions_ = p.descriptor.size();\
	}\
	\
	virtual void \
	copyToFloatArray (const PointT &p, float * out) const\
	{\
		for(int i=0; i< var_nr_dimensions_; i++) out[i] = p.descriptor[i]; \
	}\
	virtual void \
	copyFloatToPoint(const float* data, PointT& p){\
		p.descriptor.resize(var_nr_dimensions_);\
		for(int i=0; i<var_nr_dimensions_;i++) p.descriptor[i]=data[i]; \
	}\
};




/* _________________________________
   |                                 |
   |        FUNCTION PROTOTYPES      |
   |_________________________________| */


/**
 * @brief estimateSpinImages for given point cloud
 *
 * @param cloud
 * @param downsampling_voxel_size
 * @param normal_estimation_radius
 * @param spin_image_width
 * @param spin_image_cos_angle
 * @param spin_image_minimum_neighbor_density
 * @param spin_image_support_lenght
 * @param spin_images_msg (OUTPUT)
 * @param subsampled_spin_image_num_keypoints
 *
 * @return 
 */
int keypoint_selection( boost::shared_ptr<pcl::PointCloud<PointT> > target_pc, 
			float uniform_sampling_size,
			boost::shared_ptr<pcl::PointCloud<PointT> > uniform_keypoints,
			boost::shared_ptr<pcl::PointCloud<int> >uniform_sampling_indices
  		    );

int estimateSpinImages(boost::shared_ptr<PointCloud<PointT> > cloud, 
		float downsampling_voxel_size, 
		float normal_estimation_radius,
		int spin_image_width,
		float spin_image_cos_angle,
		size_t spin_image_minimum_neighbor_density,
		float spin_image_support_lenght,
		boost::shared_ptr< vector <SITOV> > spin_images_msg,
		size_t subsampled_spin_image_num_keypoints
		);	
		
int estimateSpinImages2(boost::shared_ptr<PointCloud<PointT> > cloud, 
		float uniform_downsampling_voxel_size, 
		float normal_estimation_radius,
		int spin_image_width,
		float spin_image_cos_angle,
		size_t spin_image_minimum_neighbor_density,
		float spin_image_support_lenght,
		boost::shared_ptr< vector <SITOV> > spin_images_msg,
		boost::shared_ptr<pcl::PointCloud<int> >uniform_sampling_indices
		);

/* _________________________________
   |                                 |
   |        FUNCTION PROTOTYPES      |
   |_________________________________| */
/**
 * @brief differenceBetweenSpinImage : compute distance between spinimage
 * @param sp1: spinimage vector of float value (153 elements)  
 * @param sp2: spinimage vector of float value (153 elements)
 * @param difference (output) : float
 * @return 
 */		
int differenceBetweenSpinImage( SITOV sp1, 
				 SITOV sp2,
				 float &difference);	
					


int KLDifferenceBetweenSpinImage( SITOV  sp1, 
				   SITOV  sp2,
				   float &difference);


int KLdifferenceBetweenObjectViews( std::vector< SITOV> spin_images1, 
				   std::vector< SITOV> spin_images2,
				  float &difference);

/* _________________________________
   |                                 |
   |        FUNCTION PROTOTYPES      |
   |_________________________________| */
/**
 * @brief differenceBetweenObjectViews : compute distance between objectview spinimages 
 * @param spin_images1: vector of spinimage
 * @param spin_images2: vector of spinimage
 * @param difference (output) : float
 * @return 
 */		
int differenceBetweenObjectViews( vector <SITOV>  spin_images1, 
				   vector <SITOV>  spin_images2,
				  float &difference);		
			
			
/* _________________________________
   |                                 |
   |        FUNCTION PROTOTYPES      |
   |_________________________________| */
/**
 * @brief printSpinImages 
 * @param spin_images: vector of spinimages
 * @return 
 */	
			
void printSpinImages(vector <SITOV> spin_images);
	

void  visulazeObjectViewSpinImageMatlab (vector <SITOV> spin_images,  const char* Filename);


int estimateVFH(boost::shared_ptr<PointCloud<PointT> > cloud, 
		float downsampling_voxel_size, 
		float normal_estimation_radius,
		boost::shared_ptr< vector <SITOV> > viewpoint_feature_histogram_msg,
		size_t subsampled_viewpoint_feature_histogram_num_keypoints
		);

#endif

