// ############################################################################
//    
//   Created: 	1/09/2014
//   Author : 	Hamidreza Kasaei
//   Email  :	seyed.hamidreza@ua.pt
//   Purpose: 	This program follows the teaching protocol and autonomously
//		interact with the system using teach, ask and correct actions. 
// 		For each newly taught category, the average sucess of the system
// 		should be computed. To do that, the simulated teacher repeatedly 
// 		picks object views of the currently known categories from a 
// 		database and presents them to the system for checking whether 
// 		the system can recognize them. If not, the simulated teacher provides
// 		corrective feedback.
//   		
// 		This program is part of the RACE project, partially funded by the
//   		European Commission under the 7th Framework Program.
//
//   		See http://www.project-race.eu
// 
//   		(Copyright) University of Aveiro - RACE Consortium
// 
// ############################################################################

#ifndef _SIMULATED_USER_LIB_H_
#define _SIMULATED_USER_LIB_H_

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
    #include <cctype> // string manipulation
    #include <math.h> 
    #include <cstring>

    //pcl includes
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <pcl/conversions.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl_conversions/pcl_conversions.h>

    //package includes
    #include <feature_extraction/spin_image.h>
    #include <race_perception_msgs/perception_msgs.h>
    #include <object_conceptualizer/object_conceptualization.h>
    #include <rug_simulated_user/rug_simulated_user_functionality.h>
    #include <race_perception_utils/print.h>
    #include <race_perception_msgs/CompleteRTOV.h>
    #include <race_perception_db/perception_db_serializer.h>

    #include <cv_bridge/cv_bridge.h>

    


/* _________________________________
   |                                 |
   |           NAMESPACES            |
   |_________________________________| */

    using namespace pcl;
    using namespace race_perception_msgs;
    using namespace race_perception_db;


/* _________________________________
  |                                 |
  |           Defines               |
  |_________________________________| */

    typedef PointXYZRGBA PointT; // define new type.


/* _________________________________
   |                                 |
   |        FUNCTION PROTOTYPES      |
   |_________________________________| */

    int getAvergeRGBColorOfObject(boost::shared_ptr<PointCloud<PointT> > target_pc, 
                                    float &red, 
                                    float &green,
                                    float &blue);


    int rgb2hsv(float r, float g, float b, float &h, float &s, float &v);


    /**
    * @brief IntroduceNewInstance
    * @param PCDFileAddress (input parameter)
    * @param cat_id (input parameter)
    * @param track_id (input parameter)
    * @param view_id (input parameter)
    * @return 
    */
    int updateNaiveBayesModel(std::string cat_name,
			  unsigned int cat_id,
			  PrettyPrint &pp );
    
    
    
    int updateGOODNaiveBayesModel(std::string cat_name,
			       unsigned int cat_id,
			       PrettyPrint &pp);
    
    /**
    * @brief IntroduceNewInstance
    * @param PCDFileAddress (input parameter)
    * @param cat_id (input parameter)
    * @param track_id (input parameter)
    * @param view_id (input parameter)
    * @return 
    */	
//     int IntroduceNewInstance ( std::string PCDFileAddress, unsigned int cat_id, 
// 			    unsigned int track_id, unsigned int view_id
// 			    );

int IntroduceNewInstance ( std::string PCDFileAddress,
			   unsigned int cat_id, 
			   unsigned int track_id,
			   unsigned int view_id, 
			   int spin_image_width_int,
			   float spin_image_support_lenght_float,
			   size_t subsampled_spin_image_num_keypoints
		    	 );
    

int IntroduceNewInstance2 ( string database_path,
			    std::string PCDFileAddress,
			   unsigned int cat_id, 
			   unsigned int track_id,
			   unsigned int view_id, 
			   int spin_image_width_int,
			   float spin_image_support_lenght_float,
			   float uniform_sampling_size
			  );



/**
    * @brief putObjectViewSpinImagesInSpecificCategory
    * @param cat_name (input parameter)
    * @param cat_id (input parameter)
    * @param track_id (input parameter)
    * @param view_id (input parameter)
    * @param SpinImageMsg (input parameter) 
    * @param Cat_ICD (input/output parameter) 
    * @return 
    */	
    int putObjectViewSpinImagesInSpecificCategory(std::string cat_name, unsigned int cat_id, 
						unsigned int track_id, unsigned int view_id, 
						boost::shared_ptr< vector <SITOV> > SpinImageMsg,
						double &Cat_ICD
						);

    
    float contextChangeProbability (int category_number, int number_of_categories_in_dataset);

    
    /**
    * @brief inContext
    * @param context_index (input parameter)
    * @param number_of_contexts (input parameter)
    * @param overlap_between_contexts (input parameter)
    * @param number_of_categories_in_dataset (input parameter)
    * @return 
    */	

    bool inContext(int category_index, 
		    int context_index, 
		    int number_of_contexts, 
		    int overlap_between_contexts, 
		    int number_of_categories_in_dataset);
      
    bool inContext( int category_index, 
		    int context_index, 
		    int context_change_index, 
		    int number_of_categories_in_dataset );

    
    
    /**
    * @brief selectAnInstancefromSpecificCategory
    * @param category_index (input parameter)
    * @param instance_number (input/output parameter)
    * @param Instance (input/output parameter)
    * @return 
    */	
    int selectAnInstancefromSpecificCategory(unsigned int category_index, 
					    unsigned int &instance_number, 
					    string &Instance);


    /**
    * @brief generateSequence
    * @param n (input parameter)
    * @return 
    */	
    vector <int> generateSequence (int n);


    /**
    * @brief generateRrandomSequencesInstances
    * @param path (input parameter)
    * @return 
    */	
    int generateRrandomSequencesInstances (string path);


    /**
    * @brief generateRrandomSequencesCategories
    * @param RunCount (input parameter)
    * @return 
    */	
    int generateRrandomSequencesCategories (int RunCount);


    /**
    * @brief compute_precision_of_last_3n
    * @param recognition_results (input parameter)
    * @param number_of_taught_categories (input parameter)
    * @return float precision 
    */	
    float compute_precision_of_last_3n (vector <int> recognition_results, 
					int number_of_taught_categories);

    
    float computeF1OfLast3n (vector <int> recognition_results ,
				     int number_of_taught_categories);
    /**
    * @brief report_current_results
    * @param TP (input parameter)
    * @param FP (input parameter)
    * @param FN (input parameter)
    * @param fname (input parameter)
    * @param global (input parameter) 
    * @return 
    */	
    
    
    int compute_Precision_Recall_Fmeasure_of_last_3n (vector <int> recognition_results ,
				     int number_of_taught_categories, 
				     float &Precision, float &Recall, float &F1);
    
    void monitorPrecision (string precision_file, float Precision );

    void monitorF1VsLearnedCategory (string f1_vs_learned_caegory, int TP, int FP, int FN );

    
    
    void reportCurrentResults(int TP, int FP, 
				int FN, string fname, 
				bool global);

    /**
    * @brief report_category_introduced
    * @param fname (input parameter)
    * @param cat_name (input parameter) 
    * @return 
    */	
    void report_category_introduced(string fname, string cat_name);

    /**
    * @brief report_category_introduced
    * @param fname (input parameter)
    * @param Precision (input parameter) 
    * @return 
    */	
    void report_precision_of_last_3n(string fname, double Precision);

    void reportF1OfLast3n(string fname, double F1)
{
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file << "\n\t*********** F1 of last 3n **************";
	Result_file << "\n\t\t - F1 = "<< F1;
	Result_file << "\n\t********************************************";
	Result_file.close();
};
    
    
    int reportExperimentResult (vector <float> average_class_precision,
			     int number_of_stored_instances, 
			     int number_of_taught_categories,  
			     string fname, ros::Duration duration);
    

    string fixedLength (string name , size_t length);

    /**
    * @brief extractObjectName
    * @param Object_name_orginal (input parameter)
    * @return string (output parameter)
    */	
    string extractObjectNameSimulatedUser (string object_name_orginal );


    /**
    * @brief extractCategoryName
    * @param InstancePath (input parameter)
    * @return string (output parameter)
    */	
    string extractCategoryName (string InstancePath);



    /**
    * @brief introduceNewCategory
    * @param class_index (input parameter)
    * @param track_id (input/output parameter)
    * @param instance_number (input/output parameter)
    * @param fname (input parameter)
    * @return 
    */	
//     void introduceNewCategory(int class_index,
// 			    unsigned int &track_id,
// 			    unsigned int &instance_number,
// 			    string fname);
    int introduceNewCategory(int class_index,
			    unsigned int &track_id,
			    unsigned int &instance_number,
			    string fname, 
			    int spin_image_width_int,
			    float spin_image_support_lenght_float,
			    size_t subsampled_spin_image_num_keypoints
			    );
    
int introduceNewCategory2(string home_address,
			  int class_index,
			  unsigned int &track_id,
			  unsigned int &instance_number,
			  string fname, 
			  int spin_image_width_int,
			  float spin_image_support_lenght_float,
			  float uniform_sampling_size
 			);
    /**
    * @brief addObjectViewHistogramInSpecificCategory
    * @param cat_name (input parameter)
    * @param cat_id (input parameter)
    * @param track_id (input parameter)
    * @param view_id (input parameter)
    * @param objectViewHistogram (input parameter) 
    * @param pp (input parameter) 
    * @return 
    */	
    int addObjectViewHistogramInSpecificCategory(std::string cat_name, 
						unsigned int cat_id, 
						unsigned int track_id,
						unsigned int view_id,
						SITOV objectViewHistogram,
						PrettyPrint &pp );


    int addObjectViewHistogramInSpecificCategoryDeepLearning(std::string cat_name, unsigned int cat_id, 
															unsigned int track_id, unsigned int view_id, 
															SITOV objectViewHistogram , PrettyPrint &pp
															);                        

    /**
    * @brief objectRepresentationBagOfWords
    * @param cluster_center (input parameter)
    * @param object_spin_images (input parameter)
    * @param object_representation (input/output parameter)
    * @return 
    */	
    int objectRepresentationBagOfWords (vector <SITOV> cluster_center, 
					vector <SITOV> object_spin_images, 
					SITOV  &object_representation );


    int notNormalizedObjectRepresentationBagOfWords (vector <SITOV> cluster_center, 
						     vector <SITOV> object_spin_images, 
						     SITOV  &object_representation );
    
    /**
    * @brief readClusterCenterFromFile
    * @param path (input parameter)
    * @return 
    */	
    vector <SITOV>  readClusterCenterFromFile (string path);


    /**
    * @brief IntroduceNewInstanceHistogram
    * @param PCDFileAddress (input parameter)
    * @param cat_id (input parameter)
    * @param track_id (input parameter)
    * @param pp (input parameter)
    * @return 
    */	
//     int IntroduceNewInstanceHistogram ( std::string PCDFileAddress, 
// 					unsigned int cat_id, 
// 					unsigned int track_id, 
// 					unsigned int view_id,
// 					PrettyPrint &pp);

    int IntroduceNewInstanceHistogram ( std::string PCDFileAddress, 
				    unsigned int cat_id, 
				    unsigned int track_id, 
				    unsigned int view_id,
				    PrettyPrint &pp,
				    int spin_image_width_int,
				    float spin_image_support_lenght_float,
				    size_t subsampled_spin_image_num_keypoints
  				);

    
        /**
    * @brief IntroduceNewInstanceHistogramNotNormalized
    * @param PCDFileAddress (input parameter)
    * @param cat_id (input parameter)
    * @param track_id (input parameter)
    * @param pp (input parameter)
    * @return 
    */	
    int IntroduceNewInstanceHistogramNotNormalized ( std::string PCDFileAddress, 
						unsigned int cat_id, 
						unsigned int track_id, 
						unsigned int view_id,
						PrettyPrint &pp,
						int spin_image_width_int,
						float spin_image_support_lenght_float,
						size_t subsampled_spin_image_num_keypoints						 
						);
    
    
    /**
    * @brief IntroduceNewInstanceHistogramNotNormalized
    * @param PCDFileAddress (input parameter)
    * @param cat_id (input parameter)
    * @param track_id (input parameter)
    * @param view_id (input parameter)
    * @param pp (input parameter)
    * @param spin_image_width_int,
    * @param spin_image_support_lenght_float,
    * @param uniform_sampling_size
    * @return 
    */	
    int IntroduceNewInstanceHistogramNotNormalized2 ( std::string PCDFileAddress, 
						    unsigned int cat_id, 
						    unsigned int track_id, 
						    unsigned int view_id,
						    PrettyPrint &pp,
						    int spin_image_width_int,
						    float spin_image_support_lenght_float,
						    float uniform_sampling_size,
						    vector <SITOV> dictionary
						    );
    
    /**
    * @brief introduceNewCategoryHistogram
    * @param class_index (input parameter)
    * @param track_id (input parameter)
    * @param instance_number (input parameter)
    * @param fname (input parameter)
    * @param pp (input parameter) 
    * @return 
    */	
//     void introduceNewCategoryHistogram(int class_index,
// 				    unsigned int &track_id,
// 				    unsigned int &instance_number,
// 				    string fname,
// 				    PrettyPrint &pp);
    int introduceNewCategoryHistogram(int class_index,
				   unsigned int &track_id,
				   unsigned int &instance_number,
				   string fname,
				   PrettyPrint &pp,
				   int spin_image_width_int,
				   float spin_image_support_lenght_float,
				   size_t subsampled_spin_image_num_keypoints
  				);
     /**
    * @brief introduceNewCategoryHistogram
    * @param class_index (input parameter)
    * @param track_id (input parameter)
    * @param instance_number (input parameter)
    * @param fname (input parameter)
    * @param pp (input parameter) 
    * @return 
    */	
//     void introduceNewCategoryHistogramAndUpdatingNaiveBayesModel(int class_index,
// 								unsigned int &track_id,
// 								unsigned int &instance_number,
// 								string fname,
// 								PrettyPrint &pp);
    int introduceNewCategoryHistogramAndUpdatingNaiveBayesModel(int class_index,
								unsigned int &track_id,
								unsigned int &instance_number,
								string fname,
								PrettyPrint &pp,
								int spin_image_width_int,
								float spin_image_support_lenght_float,
								size_t subsampled_spin_image_num_keypoints
								);
    
    int introduceNewCategoryHistogramAndUpdatingNaiveBayesModel2(int class_index,
							    unsigned int &track_id,
							    unsigned int &instance_number,
							    string fname,
							    PrettyPrint &pp,
							    int spin_image_width_int,
							    float spin_image_support_lenght_float,
							    float uniform_sampling_size,
							    vector <SITOV> dictionary
							    );
    
    void plotSimulatedTeacherProgressInMatlab( int RunCount, float P_Threshold, string precision_file);

    void plotLocalF1VsNumberOfLearnedCategoriesInMatlab( int RunCount, float P_Threshold, string local_F1_vs_learned_category);
    
    void plotGlobalF1VsNumberOfLearnedCategoriesInMatlab( int RunCount, string global_F1_vs_learned_category);
    
    void plotNumberOfLearnedCategoriesVsIterationsInMatlab( int RunCount, string Number_of_learned_categories_vs_Iterations);
    
    void plotNumberOfStoredInstancesPerCategoryInMatlab( vector <ObjectCategory> list_of_object_category);

    
    int IntroduceNewInstanceVFH ( std::string PCDFileAddress,
				  unsigned int cat_id, 
				  unsigned int track_id,
				  unsigned int view_id
				);
    
    int introduceNewCategoryVFH (int class_index,
				 unsigned int &track_id,
				 unsigned int &instance_number,
				 string fname
				 );
    
    bool fexists(std::string filename);
    
    int reportAllExperimentalResults ( int TP, int FP, int FN,int obj_num,			    
                                      vector <float> average_class_precision,
                                      float number_of_stored_instances, 
                                      int number_of_taught_categories,
                                      string name_of_approach );
    
    int sum_all_experiments_results ( int iterations,
				  float Success_Precision		,			    
				  float average_class_precision_value,
				  float number_of_stored_instances, 
				  int number_of_taught_categories,
				  string name_of_approach );


    int average_all_experiments_results ( int total_number_of_experiments,
					  string name_of_approach);

    
    int introduceNewInstanceGOOD ( string database_path,
                                    std::string PCDFileAddress,
                                    unsigned int cat_id, 
                                    unsigned int track_id,
                                    unsigned int view_id, 
                                    int adaptive_support_lenght,
                                    double global_image_width,
                                    int threshold,
                                    int number_of_bins,
                                    PrettyPrint &pp
                                    );
    int introduceNewInstanceGOODPlusColor ( string database_path,
								std::string PCDFileAddress,
								unsigned int cat_id, 
								unsigned int track_id,
								unsigned int view_id, 
								int adaptive_support_lenght,
								double global_image_width,
								int threshold,
								int number_of_bins,
								double weight_color,
								string color_space,
								PrettyPrint &pp	);
        
    
    int introduceNewCategoryGOOD(string home_address,
                                int class_index,
                                unsigned int &track_id,
                                unsigned int &instance_number,
                                string fname, 
                                int adaptive_support_lenght,
                                double global_image_width,
                                int threshold,
                                int number_of_bins
                                );

    int introduceNewCategoryGOODPlusColor(string home_address,
                                            int class_index,
                                            unsigned int &track_id,
                                            unsigned int &instance_number,
                                            string fname, 
                                            int adaptive_support_lenght,
                                            double global_image_width,
                                            int threshold,
                                            int number_of_bins,
                                            double weight_color,
                                            string color_space = "RGB");
    
    int introduceNewCategoryGOODAndUpdatingNaiveBayesModel(string home_address,
			  int class_index,
			  unsigned int &track_id,
			  unsigned int &instance_number,
			  string fname, 
			  int adaptive_support_lenght,
			  double global_image_width,
			  int threshold,
			  int number_of_bins
 			);
    

    int IntroduceNewInstanceDeepLearningUsingGOOD ( string database_path,
												std::string PCDFileAddress,
												unsigned int cat_id, 
												unsigned int track_id,
												unsigned int view_id, 
												int adaptive_support_lenght,
												double global_image_width,
												int threshold,
												int number_of_bins,
												ros::ServiceClient deep_learning_server,
												PrettyPrint &pp );

    
    int introduceNewCategoryDeepLearningUsingGOOD(string home_address,
												int class_index,
												unsigned int &track_id,
												unsigned int &instance_number,
												string fname, 
												int adaptive_support_lenght,
												double global_image_width,
												int threshold,
												int number_of_bins, 
												ros::ServiceClient deep_learning_server
												);



    int report_all_context_change_experiments_results (int TP, int FP, int FN,int itr_num,
						    int context_change_idx,int context_change_itr,
						    vector <float> average_class_precision,
						    float number_of_stored_instances, 
						    int number_of_taught_categories,
						    string name_of_approach,
						    int total_number_of_experiments
						  );
    
    
    int read_a_number_from_file ( string pakage_name,
			       string file_name );

    int write_a_number_to_file ( string pakage_name,
                    string file_name,
                    int number );

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    int chiSquaredDistanceBetweenTwoObjectViewHistogram (SITOV objectViewHistogram1,
                                SITOV objectViewHistogram2, 
                                float &diffrence);
                                
    ////////////////////////////////////////////////////////////////////////////////////////////////////
    int chiSquaredBasedObjectCategoryDistance( SITOV target,
            vector< SITOV > category_instances,
            float &minimumDistance, 
            int &best_matched_index, 
            PrettyPrint &pp);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    int manhatanBasedObjectCategoryDistanceShapeAndColor(   SITOV target,
														vector< SITOV > category_instances,
														int number_of_bins, 
														double weight_color,
                                                        string color_space,
														float &minimumDistance,
														int &best_matched_index,
														PrettyPrint &pp);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    int reportAllExperimentalResultsGOODPlusColor (int TP, int FP, int FN,int obj_num,			    
                                                vector <float> average_class_precision,
                                                float number_of_stored_instances, 
                                                int number_of_taught_categories,
                                                string name_of_approach,
                                                string color_space, 
                                                float weight_color, 
                                                int number_of_bins
                                            );
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    int introduceNewInstanceLocalHDP(   string home_address,
                                        std::string object_path, 
                                        unsigned int cat_id, 
                                        unsigned int track_id, 
                                        unsigned int view_id,
                                        int spin_image_width_int,
                                        float spin_image_support_lenght_float,
                                        float voxel_size_for_keypoints,
                                        vector <SITOV> dictionary,
                                        ros::ServiceClient topic_modelling_server,
                                        PrettyPrint &pp );

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    int introduceNewCategoryLocalHDP(	string home_address, 
                                        int class_index,
                                        unsigned int &track_id,
                                        unsigned int &instance_number,
                                        string fname,
                                        int spin_image_width_int,
                                        float spin_image_support_lenght_float,
                                        float voxel_size_for_keypoints,
                                        vector <SITOV> dictionary,
                                        ros::ServiceClient topic_modelling_server,
                                        PrettyPrint &pp);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    bool isImageEmpty( cv_bridge::CvImagePtr cv_ptr);
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    int IntroduceNewInstanceRGBDDeepLearningUsingGOOD ( string database_path,
                                                        std::string PCDFileAddress,
                                                        unsigned int cat_id, 
                                                        unsigned int track_id,
                                                        unsigned int view_id, 
                                                        int adaptive_support_lenght,
                                                        double global_image_width,
                                                        int threshold,
                                                        int number_of_bins,
                                                        ros::ServiceClient deep_learning_server,
                                                        PrettyPrint &pp );

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    int introduceNewCategoryRGBDDeepLearningUsingGOOD( string dataset_path,
                                                    int class_index,
                                                    unsigned int &track_id,
                                                    unsigned int &instance_number,
                                                    string fname, 
                                                    int adaptive_support_lenght,
                                                    double global_image_width,
                                                    int threshold,
                                                    int number_of_bins, 
                                                    ros::ServiceClient deep_learning_server,
                                                    int k = 3);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    int introduceNewInstanceUsingAHandCraftedDescriptor( string database_path,
                                                        std::string PCDFileAddress,
                                                        unsigned int track_id,												
                                                        string object_descriptor,                                                      
                                                        int number_of_bins, 
                                                        double normal_estimation_radius,
                                                        PrettyPrint &pp );

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    int introduceNewCategoryUsingAHandCraftedDescriptor(string database_path,
                                                        int class_index,
                                                        unsigned int &track_id,
                                                        unsigned int &instance_number,
                                                        string object_descriptor,                                                      
                                                        int number_of_bins, 
                                                        double normal_estimation_radius,
                                                        string fname, 
                                                        int k = 3);

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    int introduceNewInstanceDeepLearningUsingMultiView (string dataset_path,
														std::string PCDFileAddress,
														unsigned int cat_id, 
														unsigned int track_id,
														unsigned int view_id, 
														int adaptive_support_lenght,
														double global_image_width,
														int threshold,
														int image_resolution,									
														float uniform_sampling_size,
														float radius,
														string viewpoint_setup,
														int mvcnn_alpha,
														int number_of_cameras_along_z_axis, /* n */
														int number_of_cameras_around_z_axis,												
														ros::ServiceClient deep_learning_server, 
														bool modelnet_dataset,
														PrettyPrint &pp );

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    int introduceNewCategoryUsingMultiViewDeepLearning( string dataset_path,
                                                        int class_index,
                                                        unsigned int &track_id,
                                                        unsigned int &instance_number,
                                                        int image_resolution,									
                                                        float uniform_sampling_size,
                                                        float radius,
                                                        string viewpoint_setup,
                                                        int mvcnn_alpha,
                                                        int number_of_cameras_along_z_axis, /* n */
                                                        int number_of_cameras_around_z_axis,												
                                                        ros::ServiceClient deep_learning_server, 
                                                        bool modelnet_dataset);

#endif

