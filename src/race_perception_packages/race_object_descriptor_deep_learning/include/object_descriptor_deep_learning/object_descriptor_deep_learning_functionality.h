#ifndef _OBJECT_DESCRIPTOR_LIB_H_
#define _OBJECT_DESCRIPTOR_LIB_H_


/* _________________________________
  |                                 |
  |           Defines               |
  |_________________________________| */
#ifndef _OBJECT_DESCRIPTOR_LIB_H_DEBUG_
#define _OBJECT_DESCRIPTOR_LIB_H_DEBUG_TRUE
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


//////////////////////TODO: Must be carefuly checked

//includes
#include <CGAL/Plane_3.h>
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
// #include <pcl/visualization/histogram_visualizer.h>


//system includes
#include <stdio.h>
#include <stdlib.h>

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>   
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/cloud_viewer.h>

//package includes
#include <race_perception_msgs/perception_msgs.h>
#include <race_perception_msgs/CompleteRTOV.h>

//Eigen includes
#include <Eigen/Core>

//PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <CGAL/Plane_3.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

////new added
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
//new includes from preprocessing
//ROS includes
#include <ros/ros.h>
//#include "pcl_ros/impl/transforms.hpp"
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
//#include <pcl/common/impl/transforms.hpp>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//#include "/opt/ros/fuerte/stacks/geometry/tf_conversions/include/tf_conversions/tf_eigen.h"
#include <visualization_msgs/Marker.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
//#include <../calvin_robot/katana_driver/kni/KNI_4.3.0/lib/kinematics/roboop/newmat/newmat.h>
//#include <../calvin_robot/katana_driver/kni/KNI_4.3.0/lib/kinematics/roboop/newmat/newmat.h>
#include <race_perception_utils/print.h>
#include <pcl_conversions/pcl_conversions.h>


//raceua includes
#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>
#include <race_perception_db/MsgTest.h>
#include <race_perception_utils/cycle.h>
#include <race_perception_utils/print.h>

#include <race_perception_msgs/TOVI.h>

#include <colormap_utils/colormap_utils.h>

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
typedef PointXYZRGBA T;  

typedef PointCloud<PointT> PointCloudT;
typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;

// PointCloudPtrT cloud_reference;
// PointCloudPtrT initial_cloud_ref;

#define SORT_BY_ENTROPY_AND_1D_VARIANCE			0
#define SORT_BY_ENTROPY_AND_2D_STDDEV_SUM			1
#define SORT_BY_DISTINCTIVENESS				2
#define SORT_BY_ENTROPY_AND_2D_VARIANCE_SUM		3
#define SORT_BY_ENTROPY_AND_AVERAGE_DISTANCE		4 

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





int compute_bounding_box_dimensions(boost::shared_ptr<PointCloud<PointT> > pc, 
				      geometry_msgs::Vector3& dimensions);



// template <typename T>
int project_pc_to_plane(boost::shared_ptr<pcl::PointCloud<T> > pc_in, 
			boost::shared_ptr<pcl::ModelCoefficients> coefficients, 
			boost::shared_ptr<pcl::PointCloud<T> > pc_out);



void print_tf_transform(tf::Transform* tf, 
			 std::string s);


// template <typename T>
void printObjectViewHistograms ( vector < vector <T> > object_view_histograms);

// template <typename T>
void printHistogram ( vector <T>  histogram, string plane_name);


void convert2DhistogramTo1Dhisftogram(vector <vector <int> >  _2DHistogram, 
				      vector <int>  &histogram);

void convert2DhistogramTo1Dhisftogram(vector <vector <float> >  _2DHistogram, vector <float>  &histogram);



// template <typename T>
int XYsignDisambiguation(boost::shared_ptr<pcl::PointCloud<T> >  XOY_projected_view,
			int threshold, int &sign );

// template <typename T>
int XsignDisambiguation(boost::shared_ptr<pcl::PointCloud<T> >  XoZ_projected_view,
			int threshold, int &sign );

// template <typename T>
int YsignDisambiguation(boost::shared_ptr<pcl::PointCloud<T> >  YoZ_projected_view,
			int threshold, int &sign );


// template <typename T>
int YOZ2DObjectHistogram( boost::shared_ptr<pcl::PointCloud<T> >  YOZ_projected_view,
			  double largest_side, 
			  int number_of_bins, 
			  int sign,
			  vector < vector<int> > &YOZ_histogram);


// template <typename T>
int XOZ2DObjectHistogram( boost::shared_ptr<pcl::PointCloud<T> >  XOZ_projected_view,
			  double largest_side, 
			  int number_of_bins,
			  int sign,
			  vector < vector<int> > &XOZ_histogram);

// template <typename T>
int XOY2DObjectHistogram( boost::shared_ptr<pcl::PointCloud<T> >  XOY_projected_view,
			  double largest_side, 
			  int number_of_bins, 
			  int sign,
			  vector < vector<int> > &XOY_histogram);


int normalizingHistogram(vector <int> histogram, 
			  vector <float> &normalized_histogram);



int viewpointEntropy(vector <float> normalized_histogram,
		     float &entropy);


int viewpointEntropyNotNormalized(vector <int> not_normalized_histogram,
		    float &entropy);

int findMaxViewPointsEntropy( vector <float> view_point_entropy, 
			      int &index);

	        
int kullbackLiebler ( vector <float>  Ptheta,
		      vector <float>  Qtheta,
		      float &similarity )  ;

int avrageHistograms( vector< float> histogram1,
			vector< float> historam2,
			vector< float> historam3,
			vector< float> &average);		      


int meanOfHistogram(vector< float> histogram, 
		    float &mean);

int varianceOfHistogram(vector< float> histogram, 
			float mean, 
			float &variance);

int subtractTowHistogram(vector< float> histogram1,
			vector< float> historam2,
			vector< float> &subtract);


int objectViewHistogram( int maximum_entropy_index,
			vector <float> view_point_entropy,
			vector< vector<float> >normalized_projected_views,
			vector< float> &sorted_normalized_projected_views,
			string &std_name_of_sorted_projected_plane /*debug*/
		      );

int compute_largest_side_of_bounding_box(geometry_msgs::Vector3 dimensions,  double &largest_side );

int downSampling ( boost::shared_ptr<PointCloud<PointT> > cloud, 		
		  float downsampling_voxel_size, 
		  boost::shared_ptr<PointCloud<PointT> > downsampled_pc);

int compuet_object_description( boost::shared_ptr<pcl::PointCloud<T> > target_pc,
				 int adaptive_support_lenght,
				 double global_image_width,
				 int threshold,
				 int number_of_bins,
				boost::shared_ptr<pcl::PointCloud<T> > &pca_object_view,
				Eigen::Vector3f &center_of_bbox,
				vector < boost::shared_ptr<pcl::PointCloud<T> > > &vector_of_projected_views, 
				double &largest_side, 
				int &sign,
				vector <float> &view_point_entropy,
				string &std_name_of_sorted_projected_plane,
				vector< float > &object_description);


int compuet_object_description_real_demo( boost::shared_ptr<pcl::PointCloud<T> > target_pc,
				 int adaptive_support_lenght,
				 double global_image_width,
				 int threshold,
				 int number_of_bins,
				 geometry_msgs::Vector3 dimensions,
				 vector < boost::shared_ptr<pcl::PointCloud<T> > > &vector_of_projected_views, 
				 double &largest_side, 
				 int &sign,
				 vector <float> &view_point_entropy,
				 string &std_name_of_sorted_projected_plane,
				 vector< float > &object_description )	;

int compuet_object_description_2D_variance( boost::shared_ptr<pcl::PointCloud<T> > target_pc,
					      int adaptive_support_lenght,
					      double global_image_width,
					      int threshold,
					      int number_of_bins,
					      boost::shared_ptr<pcl::PointCloud<T> > &pca_object_view,
					      Eigen::Vector3f &center_of_bbox,
					      vector < boost::shared_ptr<pcl::PointCloud<T> > > &vector_of_projected_views, 
					      double &largest_side, 
					      int &sign,
					      vector <float> &view_point_entropy,
					      string &std_name_of_sorted_projected_plane,
					      vector< float > &object_description, 
					      int sorting_criterion);


int compuet_object_description_for_grasp( boost::shared_ptr<pcl::PointCloud<T> > pca_object_view,
					  int number_of_bins,
					  vector< float > &object_description );
#endif

