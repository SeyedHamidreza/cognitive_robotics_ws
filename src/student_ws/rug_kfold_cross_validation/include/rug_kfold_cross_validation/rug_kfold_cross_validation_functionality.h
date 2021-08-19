#ifndef _CROSS_VALIDATION_LIB_H_
#define _CROSS_VALIDATION_LIB_H_

/*________________________________
|                                 |
|           INCLUDES              |
|_________________________________| */

//ROS, Boost, and Eigen includes
#include <ros/ros.h>
#include <boost/make_shared.hpp>
#include <Eigen/Core>


//PCL includes
#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/esf.h>
#include <pcl/features/gfpfh.h>
#include <pcl/features/vfh.h>

//GRSD needs PCL 1.8.0
#include <pcl/features/grsd.h>

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
#include <object_descriptor/object_descriptor_functionality.h>

//Gi includes
#include <pluginlib/class_list_macros.h> 
#include <stdio.h>
#include <race_perception_db/perception_db_serializer.h>
#include <race_perception_utils/cycle.h>
#include <race_perception_utils/print.h>


/* _______________________________
|                                 |
|           NAMESPACES            |
|_________________________________| */

using namespace pcl;
using namespace race_perception_msgs;
using namespace race_perception_db;
using namespace race_perception_utils;

/* ________________________________
|                                 |
|        FUNCTION PROTOTYPES      |
|_________________________________| */

int kFoldTrainAndTestDataGenerator( int K_fold, 
                                    int iteration, 
                                    vector<vector<SITOV> > categories_instances,
                                    vector<vector<SITOV> > &categories_instances_train,
                                    vector<vector<SITOV> > &categories_instances_test);

//////////////////////////////////////////////////////////////////////////////////////////////////////
void multiplyVectorElementsByAScalarValue ( vector <float> &vector_list, double weight);

//////////////////////////////////////////////////////////////////////////////////////////////////////
vector <float> normalization (vector <float> hist);

////////////////////////////////////////////////////////////////////////////////////////////////////
string extractObjectName (string object_name_orginal );

////////////////////////////////////////////////////////////////////////////////////////////////////
string extractFileName (string object_name_orginal);

/////////////////////////////////////////////////////////////////////////////////////////////////////
string extractCategoryName (string instance_path);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int addObjectViewHistogramInSpecificCategory(std::string cat_name, unsigned int cat_id, 
												unsigned int track_id, 
												unsigned int view_id, 
												SITOV objectViewHistogram, 
												PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int addObjectViewHistogramInSpecificCategoryDeepLearning(std::string cat_name, unsigned int cat_id, 
														unsigned int track_id, unsigned int view_id, 
														SITOV objectViewHistogram, 
														PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizeObjectViewSpinImagesInSpecificCategory(std::string cat_name, unsigned int cat_id, 
														unsigned int track_id, unsigned int view_id, 
														vector <SITOV> SpinImageMsg,
														PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int deleteObjectViewHistogramFromSpecificCategory(	std::string cat_name, unsigned int cat_id, 
													int track_id,  int view_id,
													SITOV objectViewHistogram,
													PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int crossValidationDataCRC(int K_fold, int iteration , string home_address);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int modelNetTrainTestData(string home_address);

/////////////////////////////////////////////////////////////////////////////////////////////////////
void delelteAllSITOVFromDB ();

/////////////////////////////////////////////////////////////////////////////////////////////////////
void delelteAllRVFromDB ();

/////////////////////////////////////////////////////////////////////////////////////////////////////
int deconceptualizingAllTrainData();

/////////////////////////////////////////////////////////////////////////////////////////////////////
 int deleteObjectViewFromSpecificCategory(std::string cat_name, unsigned int cat_id, 
										 int track_id,  int view_id,
										 vector <SITOV> SpinImageMsg , PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
vector <int> generateSequence (int n);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesInstances ( string path,
				      					string home_address);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesCategories ( string home_address, 
										 int number_of_object_per_category);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesCategoriesKfold ( string home_address, 
											 int number_of_object_per_category);

/////////////////////////////////////////////////////////////////////////////////////////////////////
void reportCurrentResults(int TP, int FP, int FN,string fname, bool global);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int addingGaussianNoise( boost::shared_ptr<PointCloud<PointT> > input_pc, 
						 double standard_deviation,
						 boost::shared_ptr<PointCloud<PointT> > output_pc);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int addingGaussianNoiseXYZL( boost::shared_ptr<PointCloud<pcl::PointXYZL> > input_pc, 
							 double standard_deviation,
							 boost::shared_ptr<PointCloud<pcl::PointXYZL> > output_pc);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int downSampling( boost::shared_ptr<PointCloud<PointT> > cloud, 		
				  double downsampling_voxel_size, 
				  boost::shared_ptr<PointCloud<PointT> > downsampled_pc);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int downSamplingXYZL( boost::shared_ptr<PointCloud<pcl::PointXYZL>  > cloud, 		
					  float downsampling_voxel_size, 
					  boost::shared_ptr<PointCloud<pcl::PointXYZL> > downsampled_pc);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int estimateViewpointFeatureHistogram( boost::shared_ptr<PointCloud<PointT> > cloud, 
									   float normal_estimation_radius,
									   pcl::PointCloud<pcl::VFHSignature308>::Ptr &vfhs);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingVFHTrainData( int &track_id, 
								 PrettyPrint &pp,
								 string home_address, 
								 float normal_estimation_radius);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingVFHDownSampledTrainData( int &track_id, 
											PrettyPrint &pp,
											string home_address, 
											float normal_estimation_radius,
											float downsampling_voxel_size);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int estimateESFDescription (boost::shared_ptr<PointCloud<PointT> > cloud, 
			     			pcl::PointCloud<pcl::ESFSignature640>::Ptr &esfs);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingESFTrainData( int &track_id, 
								 PrettyPrint &pp,
								 string home_address);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingESFDownSampledTrainData( int &track_id, 
											PrettyPrint &pp,
											string home_address,
											float downsampling_voxel_size);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int estimateGRSDDescription( boost::shared_ptr<PointCloud<PointT> > cloud, 
							 float normal_estimation_radius,
							 pcl::PointCloud<pcl::GRSDSignature21>::Ptr &grsds);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGRSDTrainData( int &track_id, 
								  PrettyPrint &pp,		  
								  string home_address,
								  float normal_estimation_radius);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGRSDDownSampledTrainData( int &track_id, 
											 PrettyPrint &pp,
											 string home_address,
											 float normal_estimation_radius,
											 float downsampling_voxel_size);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int estimateGFPFH(boost::shared_ptr<PointCloud<pcl::PointXYZL> > cloud, 
				   pcl::PointCloud<pcl::GFPFHSignature16>::Ptr &gfpfhs);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGFPFHTrainData( int &track_id, 
									PrettyPrint &pp,
									string home_address);


/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGFPFHDownSampledTrainData( int &track_id, 
											  PrettyPrint &pp,
											  string home_address, 
											  float downsampling_voxel_siz );

/////////////////////////////////////////////////////////////////////////////////////////////////////													
int conceptualizingDeepLearningAndGoodDescriptor( int &track_id, 
													PrettyPrint &pp,
													string home_address, 
													int adaptive_support_lenght,
													double global_image_width,
													int threshold,
													int number_of_bins,
													ros::ServiceClient deep_learning_server, 
													bool downsampling, 
													double downsampling_voxel_size,
													bool modelnet_dataset);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingDeepLearningBasedOnRGBAndOrthographicImages( int &track_id, 
																string dataset_path, 																
																ros::ServiceClient deep_learning_server, 
																int number_of_bins,																
																PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingDeepLearningAndGoodDescriptorPlusDataAugmentation( int &track_id, 
																		PrettyPrint &pp,
																		string home_address, 
																		int adaptive_support_lenght,
																		double global_image_width,
																		int threshold,
																		int number_of_bins,
																		ros::ServiceClient deep_learning_server, 
																		bool modelnet_dataset);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGOODTrainData( int &track_id, 
								  PrettyPrint &pp,
								  string home_address, 
							 	  int adaptive_support_lenght,
								  double global_image_width,
								  int threshold,
								  int number_of_bins);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGOODDownSampledTrainData( int &track_id, 
											 PrettyPrint &pp,
											 string home_address, 
											 int adaptive_support_lenght,
											 double global_image_width,
											 int threshold,
											 int number_of_bins,
											 float downsampling_voxel_size );

							
/////////////////////////////////////////////////////////////////////////////////////////////////////
// void writeToFile (string file_name, float value );

/////////////////////////////////////////////////////////////////////////////////////////////////////
bool fexists(std::string filename);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int numberOfPerformedExperiments(string name_of_approach, int &exp_num);

//////////////////////////////////////////////////////////////////////////////////////////////////////
int chiSquaredDistanceBetweenTwoObjectViewHistogram( SITOV objectViewHistogram1,
													 SITOV objectViewHistogram2, 
													 float &diffrence);

////////////////////////////////////////////////////////////////////////////////////////////////////
int chiSquaredBasedObjectCategoryDistance( 	SITOV target,
											vector< SITOV > category_instances,
											float &minimumDistance, 
											int &best_matched_index, 
											PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int euclideanBasedObjectCategoryDistance(	SITOV target,
											vector< SITOV > category_instances,
											float &minimumDistance, 
											int &best_matched_index, 
											PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
int kLBasedObjectCategoryDistance(  SITOV target,
									vector< SITOV > category_instances,
									float &minimumDistance, 
									int &best_matched_index, 
									PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
void confusionMatrixGenerator( string true_category, string predicted_category, 
								std::vector<string> map_category_name_to_index,
								std::vector< std::vector <int> > &confusion_matrix );

/////////////////////////////////////////////////////////////////////////////////////////////////////
void findClosestCategory( vector<float> object_category_distances,
						  int &cat_index, 
						  float &mindist, 
						  PrettyPrint &pp, 
						  float &sigma_distance);

/////////////////////////////////////////////////////////////////////////////////////////////////////
vector <float> objectCategoryDistance( SITOV target,vector< SITOV > category_instances,
										float &minimumDistance, 
										int &best_matched_index,
										string distance_function,
										PrettyPrint &pp);

/////////////////////////////////////////////////////////////////////////////////////////////////////
struct KNN_struct{ vector <float> distances; string category_name; };
int knnClassification(int K, vector<KNN_struct> data, float &minimum_distance, string &win_category);

/////////////////////////////////////////////////////////////////////////////////////////////////////
void plotConfusionMatrix(std::vector< std::vector <int> > confusion_matrix,
						 std::vector<string> map_category_name_to_index, 
						 string name_of_approach);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// cv::Mat processingDepthImageWithouthFiltering ( cv::Mat img, int resolution, bool debug=false);

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename X>
void convertAll2DHistogramsTo1DVector(vector<vector<vector<X> > > all_2D_histograms,
									  vector<float> &all_1D_histograms);

////////////////////////////////////////////////////////////////////////////////////////////////////
int reportAllHandCraftedExperiments( int TP, int FP, int FN, 
									double avg_class_accuracy, 
									string object_descriptor, 
									double descriptor_param, 
									int K,
									string distance_function, 										 
									double duration_sec, string name_of_approach);

////////////////////////////////////////////////////////////////////////////////////////////////////
int reportAllDeepTransferLearningExperiments (  int TP, int FP, int FN,
											    string dataset,
											    int orthographic_image_resolution, 
												string deep_learning_architecture,											
												string pooling_function,
												string distance_function, 
												int K,
												double avg_class_accuracy, 
												string name_of_approach, 
												double duration_sec);
#endif

