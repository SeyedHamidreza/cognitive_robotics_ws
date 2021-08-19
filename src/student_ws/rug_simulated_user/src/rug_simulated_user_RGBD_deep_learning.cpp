// ############################################################################
//    
//   Created: 	1/05/2014
//   Author : 	Hamidreza Kasaei
//   Email  :	seyed.hamidreza@ua.pt
//   Purpose: 	(Purely instance based learning approach)This program follows
//   		the teaching protocol and autonomously interact with the system
//   		using teach, ask and correct actions. For each newly taught category,
//   		the average sucess of the system should be computed. To do that, 
//   		the simulated teacher repeatedly picks object views of the currently
//   		known categories from a database and presents them to the system for 
//   		checking whether the system can recognize them. If not, the simulated
//   		teacher provides corrective feedback.
//   		
// 		This program is part of the RACE project, partially funded by the
//   		European Commission under the 7th Framework Program.
//
//   		See http://www.project-race.eu
// 
//   		(Copyright) University of Aveiro - RACE Consortium
// 
// ############################################################################

/* _________________________________
   |                                 |
   |          RUN SYSTEM BY          |
   |_________________________________| */
   
//rm -rf /tmp/pdb
//roslaunch race_simulated_user simulated_user.launch

/* _________________________________
  |                                 |
  |             INCLUDES            |
  |_________________________________| */
  
//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
//ros includes 
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <object_descriptor/object_descriptor_functionality.h>

#include <pluginlib/class_list_macros.h> 
#include <stdio.h>
#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>

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
#include <rug_kfold_cross_validation/rug_kfold_cross_validation_functionality.h>


// #include <race_deep_learning_feature_extraction/vgg16_model.h>
#include <rug_deep_feature_extraction/deep_representation.h>


#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <CGAL/Plane_3.h>
#include "std_msgs/Float64.h"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

/* _________________________________
  |                                 |
  |            NameSpace            |
  |_________________________________| */

using namespace pcl;
using namespace std;
using namespace ros;
using namespace race_perception_utils;

typedef pcl::PointXYZRGBA PointT;

/* _______________________________
|                                 |
|        Global Parameters        |
|_________________________________| */

string name_of_approach = "cognitive_robotics_course"; 

//// dataset
string dataset_path = ""; 
int number_of_categories = 51;
bool random_sequence_generator = true;


//// deep learning parameters
int  orthographic_image_resolution = 50; 
string deep_learning_service = "/orthographicNet_service";
string base_network = "mobileNetV2";
string pooling_function = "MAX";
std::string distance_function = "chiSquared";
int K_for_KNN = 3;


///// do not change these params
//simulated user parameters
double protocol_threshold = 0.67;  
int user_sees_no_improvment_const = 100;
int window_size = 3;
double uniform_sampling_size = 0.03;
double recognition_threshold = 200;

bool downsampling = false;
bool image_normalization = true;
bool multiviews = true;
bool max_pooling = true;

int  off_line_flag  = 1;
int  adaptive_support_lenght = 1;
double global_image_width =0.5;
int sign = 1;
int threshold = 10;	 
double downsampling_voxel_size = 0.001;

// because of updating memory in evaluation fuction, deep_learning_server should be gloabal.
ros::ServiceClient deep_learning_server;

/* _______________________________
|                                 |
|         Global Variable         |
|_________________________________| */

PerceptionDB* _pdb;
typedef pcl::PointXYZRGBA PointT;
unsigned int cat_id = 1;
unsigned int track_id =1;
unsigned int view_id = 1;
string InstancePathTmp= "";
std::string evaluation_file, evaluationTable, precision_file, local_f1_vs_learned_category, f1_vs_learned_category;
std::ofstream summary_of_experiment , PrecisionMonitor, local_f1_vs_category, f1_vs_category, NumberofFeedback , category_random, instances_random, category_introduced;
int TP = 0, FP= 0, FN = 0, tp_tmp = 0, fp_tmp = 0, fn_tmp = 0, obj_num = 0, number_of_instances = 0;
vector <int> recognition_results; // we coded 0: continue(correctly detect unkown object)
								  // 1: TP , 2: FP , 3: FN , 4: FP and FN
				  

void evaluationFunction(const race_perception_msgs::RRTOV &result)
{
    PrettyPrint pp;
    // ROS_INFO("ground_truth_name = %s", result.ground_truth_name.c_str());
    string instance_path = result.ground_truth_name.substr(dataset_path.size(),result.ground_truth_name.size());
	string tmp = instance_path;
	string true_cat = extractCategoryName(tmp);

    ROS_INFO("ground_truth = %s", true_cat.c_str());

    std:: string object_name;
    object_name = extractObjectNameSimulatedUser (result.ground_truth_name);
    pp.info(std::ostringstream().flush() << "extractObjectName: " << object_name.c_str()); 
    
    obj_num++;
    pp.info(std::ostringstream().flush() << "track_id="<< result.track_id << "\tview_id=" << result.view_id);
          
    float minimum_distance = result.minimum_distance;
 
    string predicted_cat;
    predicted_cat= result.recognition_result.c_str();   
    pp.info(std::ostringstream().flush() << "[-]object_name: "<< object_name.c_str());
    pp.info(std::ostringstream().flush() << "[-]true_category: "<<true_cat.c_str());
    pp.info(std::ostringstream().flush() << "[-]predicted_category: " << predicted_cat.c_str());
	true_cat = fixedLength (true_cat , 15); // to have a pretty report, while the length of given string is less than 15, we add space at the end of the string
	predicted_cat = fixedLength (predicted_cat , 15); // to have a pretty report, while the length of given string is less than 15, we add space at the end of the string

    char unknown[] = "unknown";
   
    summary_of_experiment.open( evaluation_file.c_str(), std::ofstream::app);    
    summary_of_experiment.precision(3);
   
    if ((strcmp(true_cat.c_str(),unknown)==0) && (strcmp(predicted_cat.c_str(),unknown)==0))
    { 	
		recognition_results.push_back(0);// continue
		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "0\t0\t0"<< "\t\t"<< minimum_distance;
		summary_of_experiment << "\n----------------------------------------------------------------------------------------------------------------------------------------";
    }
    else if ((strcmp(true_cat.c_str(),predicted_cat.c_str())==0))
    { 
		TP++;
		tp_tmp++;
		recognition_results.push_back(1);
		
		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "1\t0\t0" << "\t\t"<< minimum_distance;
		summary_of_experiment << "\n----------------------------------------------------------------------------------------------------------------------------------------";
    }
    else if ((strcmp(true_cat.c_str(),unknown)==0) && (strcmp(predicted_cat.c_str(),unknown)!=0))
    { 	
		FP++; 
		fp_tmp++;
		recognition_results.push_back(2);

		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "0\t1\t0"<< "\t\t"<< minimum_distance;
		summary_of_experiment << "\n----------------------------------------------------------------------------------------------------------------------------------------";
    }
    else if ((strcmp(true_cat.c_str(),unknown)!=0) && (strcmp(predicted_cat.c_str(),unknown)==0))
    { 	
		FN++;
		fn_tmp++;
		recognition_results.push_back(3);

		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "0\t0\t1"<< "\t\t"<< minimum_distance;
		summary_of_experiment << "\n========================================================================================================================================";

		number_of_instances ++;
	
		introduceNewInstanceRGBDDeepLearningUsingGOOD ( dataset_path,
													instance_path,
													cat_id, track_id, view_id,
													adaptive_support_lenght,
													global_image_width,
													threshold,
													orthographic_image_resolution,
													deep_learning_server,
													pp );
		
		track_id++;
		pp.info(std::ostringstream().flush() << "[-]Category Updated");
    }
    else if ((strcmp(true_cat.c_str(),predicted_cat.c_str())!=0))
    {  	
		FP++; FN++;
		fp_tmp++; fn_tmp++;    
		recognition_results.push_back(4);

		summary_of_experiment << "\n"<<obj_num<<"\t"<<object_name <<"\t\t"<< true_cat <<"\t\t"<< predicted_cat <<"\t\t"<< "0\t1\t1"<< "\t\t"<< minimum_distance;
		summary_of_experiment << "\n========================================================================================================================================";

		number_of_instances++;

		introduceNewInstanceRGBDDeepLearningUsingGOOD ( dataset_path,
													instance_path,
													cat_id, track_id, view_id,
													adaptive_support_lenght,
													global_image_width,
													threshold,
													orthographic_image_resolution,
													deep_learning_server,
													pp );

		track_id++;
		pp.info(std::ostringstream().flush() << "[-]Category Updated");
    }
    summary_of_experiment.close();
    summary_of_experiment.clear();
    pp.printCallback();
}

int main(int argc, char** argv)
{
   
	/* __________________________________
	|                                   |
	|  Creating a folder for each RUN   |
	|___________________________________| */
	int experiment_number = 1;
	string system_command= "mkdir "+ ros::package::getPath("rug_simulated_user")+ "/result/experiment_1";
	system( system_command.c_str());

	PrettyPrint pp; // pp stands for pretty print


	precision_file = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/PrecisionMonitor.txt";
	PrecisionMonitor.open (precision_file.c_str(), std::ofstream::trunc);
	PrecisionMonitor.precision(4);
	PrecisionMonitor.close();

	local_f1_vs_learned_category = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/local_f1_vs_learned_category.txt";
	local_f1_vs_category.open (local_f1_vs_learned_category.c_str(), std::ofstream::trunc);
	local_f1_vs_category.precision(4);
	local_f1_vs_category.close();
	
	f1_vs_learned_category = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/f1_vs_learned_category.txt";
	f1_vs_category.open (f1_vs_learned_category.c_str(), std::ofstream::trunc);
	f1_vs_category.precision(4);
	f1_vs_category.close();
		
	ros::init (argc, argv, "EVALUATION");
	ros::NodeHandle nh;
	
	// initialize perception database 
	_pdb = race_perception_db::PerceptionDB::getPerceptionDB(&nh); //initialize the database class_list_macros
	string name = nh.getNamespace();
	
	/* _____________________________________
	|                                       |
	|    read prameters from launch file    |
	|_______________________________________| */

    // read name_of_approach parameter
	nh.param<std::string>("/perception/name_of_approach", name_of_approach, name_of_approach);

	// read database parameter
	nh.param<std::string>("/perception/dataset_path", dataset_path, dataset_path);
    string dataset= (dataset_path.find("restaurant_object_dataset") != std::string::npos) ? "Restaurant RGB-D Object Dataset" : "Washington RGB-D Dataset";
	number_of_categories = (dataset_path.find("restaurant_object_dataset") != std::string::npos) ? 10 : 51;
	
    // resolution of orthographic images [n x n]
    nh.param<int>("/perception/orthographic_image_resolution", orthographic_image_resolution, orthographic_image_resolution);

	// read distance_function parameter    
    nh.param<std::string>("/perception/distance_function", distance_function, "default_param");
    
    //recognition threshold
    nh.param<double>("/perception/recognition_threshold", recognition_threshold, recognition_threshold);

    // read deep_learning_service parameter
    nh.param<std::string>("/perception/deep_learning_service", deep_learning_service, deep_learning_service);

    // read base_network parameter
    nh.param<std::string>("/perception/base_network", base_network, base_network);

   	// read pooling function  ["MAX", "AVG", "APP"] 
	nh.param<string>("/perception/pooling_function", pooling_function, pooling_function);

    //K param for KNN
    nh.param<int>("/perception/K_for_KNN", K_for_KNN, K_for_KNN);

	/* _____________________________________
	|                                       |
	|   	Do not change these params  	|
	|_______________________________________| */

    // read image_normalization parameter 0 = FLASE, 1 = TRUE 
    nh.param<bool>("/perception/image_normalization", image_normalization, image_normalization);
    string image_normalization_flag = (image_normalization == 0) ? "FALSE" : "TRUE";

   	// read multiviews parameter 0 = FLASE, 1 = TRUE 
    nh.param<bool>("/perception/multiviews", multiviews, multiviews);
    string multiviews_flag = (multiviews == 0) ? "FALSE" : "TRUE";

    // read downsampling parameter 0 = FLASE, 1 = TRUE 
    nh.param<bool>("/perception/downsampling", downsampling, downsampling);
    string downsampling_flag = (downsampling == 0) ? "FALSE" : "TRUE";

    // read downsampling_voxel_size parameter
    nh.param<double>("/perception/downsampling_voxel_size", downsampling_voxel_size, downsampling_voxel_size);
    //downsampling_voxel_size = downsampling_voxel_size * 0.02;

	//read simulated teacher parameters
	nh.param<double>("/perception/protocol_threshold", protocol_threshold, protocol_threshold);
	nh.param<int>("/perception/user_sees_no_improvment_const", user_sees_no_improvment_const, user_sees_no_improvment_const);
	nh.param<int>("/perception/window_size", window_size, window_size);	

	//recognition threshold
	nh.param<double>("/perception/recognition_threshold", recognition_threshold, recognition_threshold);

    /// internal params of orthographic image generator
    nh.param<double>("/perception/global_image_width", global_image_width, global_image_width);		
    nh.param<int>("/perception/adaptive_support_lenght", adaptive_support_lenght, adaptive_support_lenght);
    nh.param<int>("/perception/threshold", threshold, threshold);

   // read random_sequence_generator parameter   
    nh.param<bool>("/perception/random_sequence_generator", random_sequence_generator, random_sequence_generator);
	string random_sequence_generator_flag = (random_sequence_generator == 0) ? "FALSE" : "TRUE";

	evaluation_file = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/summary_of_experiment.txt";
	summary_of_experiment.open (evaluation_file.c_str(), std::ofstream::out);
	
	summary_of_experiment  <<"system configuration:" 
			<< "\n\t-experiment_name = " << name_of_approach
			<< "\n\t-name_of_dataset = " << dataset //TODO make a parameter for this 
			<< "\n\t-number_of_category = "<< number_of_categories // 51 for Washington RGB-D Dataset
			<< "\n\t-random_sequence_generator = " << random_sequence_generator_flag			
			<< "\n\t-distance_function = "<< distance_function  
			<< "\n\t-orthographic_image_resolution = "<< orthographic_image_resolution
			<< "\n\t-name_of_service= " << deep_learning_service
			<< "\n\t-base_network= " << base_network
			<< "\n\t-downsampling = " << downsampling_flag
			<< "\n\t-downsampling_voxel_size = " << downsampling_voxel_size
			<< "\n\t-image_normalization = " << image_normalization_flag
			<< "\n\t-multiviews = " << multiviews_flag
			<< "\n\t-pooling_function = " << pooling_function
			<< "\n\t-K param for KNN [1, 3, 5, ...] = " << K_for_KNN
			<< "\n\t-simulated_user_threshold = " << protocol_threshold 			
			<< "\n\t-other parameters = " << "if you need to define more params, please add them here."
			<< "\n-----------------------------------------------------------------------------------------------------------------------------------------\n\n";


	summary_of_experiment << "\n\nNo."<<"\tobject_name" <<"\t\t\t\t"<< "ground_truth" <<"\t\t"<< "prediction"<< "\t\t"<< "TP" << "\t"<< "FP"<< "\t"<< "FN \t\tdistance";
	summary_of_experiment << "\n-----------------------------------------------------------------------------------------------------------------------------------------";
	summary_of_experiment.close();


	/* _______________________________
	|                                 |
	|    Randomly select categories   |
	|_________________________________| */
	if (random_sequence_generator)
 	{
		generateRrandomSequencesCategories(experiment_number);
	}	
	
	string category_introduced_txt = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/Category_Introduced.txt";
	category_introduced.open (category_introduced_txt.c_str(), std::ofstream::out);

	/* _____________________________
    |                               |
    |     create a client server    |
    |_______________________________| */
    deep_learning_server = nh.serviceClient<rug_deep_feature_extraction::deep_representation>(deep_learning_service);

    pp.info(std::ostringstream().flush() <<"deep_learning based simulated user -> Hello World");

	/* _______________________________
	|                                |
	|         Initialization         |
	|________________________________| */

	string instance_path= "";	
	int class_index = 1; // class index
	unsigned int instance_number = 1;
	cat_id = 1;  // it is a constant to create a key for each category <Cat_Name><Cat_ID>
	track_id = 1;
	view_id = 1; // it is a constant to create key for categories <TID><VID> (since the database samples were
		         // collected manually, each track_id has exactly one view)

	vector <unsigned int> instance_number2;
	for (int i = 0; i < number_of_categories ; i++)
	{
	    instance_number2.push_back(1);
	}
	    
	int number_of_taught_categories = 0;

	
	//start tic	
	ros::Time begin_process = ros::Time::now(); 
	
	/* ______________________________
	|                                |
	|        Introduce category      |
	|________________________________| */
	introduceNewCategoryRGBDDeepLearningUsingGOOD(  dataset_path, 
													class_index,track_id,instance_number2.at(class_index-1), 
													evaluation_file,
													adaptive_support_lenght,
													global_image_width,
													threshold,
													orthographic_image_resolution,
													deep_learning_server, 
													K_for_KNN);
		
	number_of_instances += 3; // we use three instances to initialize a category
	number_of_taught_categories ++;
	category_introduced << "1\n";
	

	vector <ObjectCategory> list_of_object_category = _pdb->getAllObjectCat();
	while (list_of_object_category.size() == 0)
	{
		list_of_object_category = _pdb->getAllObjectCat();
		ros::Duration(0.2).sleep(); // sleep for 0.2 a second
	}

	/* ______________________________
	|                                |
	|        Simulated Teacher       |
	|________________________________| */
	
	float precision = 0;
	float recall = 0;
	float f1 =0;
	vector <float> average_class_precision;
	
	while ( class_index < number_of_categories)  // one category is already taught above
	{
	    class_index ++; // class index
	    instance_path = "";
      
	    if ( introduceNewCategoryRGBDDeepLearningUsingGOOD( dataset_path, 
															class_index,track_id, instance_number2.at(class_index-1),
															evaluation_file,
															adaptive_support_lenght,
															global_image_width,
															threshold,
															orthographic_image_resolution, 
															deep_learning_server, 
															K_for_KNN) == -1)  
		{
			ROS_INFO("\t\t[-] *NOTE: the experiment is terminated because there is not enough test data to continue the evaluation");
			summary_of_experiment.open( evaluation_file.c_str(), std::ofstream::app);   
			summary_of_experiment << "\n-----------------------------------------------------------------------------------------------------------------------------------------";
			summary_of_experiment << " *NOTE: the experiment is terminated because there is not enough test data to continue the evaluation";
			summary_of_experiment << "\n-----------------------------------------------------------------------------------------------------------------------------------------";
			summary_of_experiment.close();

			ros::Duration duration = ros::Time::now() - begin_process;
			reportCurrentResults( TP, FP, FN, evaluation_file, true);

			reportExperimentResult( average_class_precision,
									number_of_instances, 
									number_of_taught_categories,  
									evaluation_file, duration);
						
			reportAllExperimentalResults( TP, FP, FN, obj_num,			    
											average_class_precision,
											number_of_instances, 
											number_of_taught_categories,
											name_of_approach );

			category_introduced.close();
			
			monitorF1VsLearnedCategory( f1_vs_learned_category, TP, FP, FN);

			plotSimulatedTeacherProgressInMatlab( experiment_number, protocol_threshold, precision_file);
			//plotLocalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, protocol_threshold, local_f1_vs_learned_category.c_str());
			plotGlobalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, f1_vs_learned_category.c_str());
			plotNumberOfLearnedCategoriesVsIterationsInMatlab( experiment_number, category_introduced_txt.c_str());
			plotNumberOfStoredInstancesPerCategoryInMatlab( list_of_object_category);

			return (0) ;
	    }

		
	    number_of_instances += 3; // we use three instances to initialize a category
	    number_of_taught_categories ++;
	    category_introduced << "1\n";
	    tp_tmp = 0; fp_tmp = 0; fn_tmp = 0;
	    
	    int k = 0; // number of classification results
	    float precision_tmp = 0;
	    precision = 0;
	    f1 = 0;
	    recall = 0;
	    unsigned int c = 1; // class index
	    int iterations = 1;
	    int iterations_user_sees_no_improvment = 0;
	    bool user_sees_no_improvement = false;  // In the current implementation, If the simulated teacher 
												// sees the precision doesn't improve in 100 iteration, then, 
												// it terminares evaluation of the system, originally, 
												// it was an empirical decision of the human instructor


	    while ( ((f1 < protocol_threshold ) or (k < number_of_taught_categories)) and (!user_sees_no_improvement) )
	    {
			// category_introduced<< "0\n";
			ROS_INFO("\n");

			ROS_INFO("\t\t[-] iteration:%i",iterations);
			ROS_INFO("\t\t[-] c:%i",c);
			ROS_INFO("\t\t[-] instance number:%i",instance_number2.at(c-1));
			//info for debug
			ROS_INFO("\t\t[-] dataset : %s", dataset_path.c_str());
			ROS_INFO("\t\t[-] number_of_categories : %i", number_of_categories);
			ROS_INFO("\t\t[-] protocol_threshold : %lf", protocol_threshold);
			ROS_INFO("\t\t[-] user_sees_no_improvment_const : %i", user_sees_no_improvment_const);
			ROS_INFO("\t\t[-] window_size : %i", window_size);

			// select an instance from an specific category
			instance_path= "";
			selectAnInstancefromSpecificCategory(c, instance_number2.at(c-1), instance_path);
			ROS_INFO("\t\t[-] test instance: %s", instance_path.c_str());
		
			// check the selected instance exist or not? 
			if (instance_path.size() < 2) 
			{
				ROS_INFO("\t\t[-] the %s file does not exist", instance_path.c_str());
				ROS_INFO("\t\t[-] number of taught categories= %i", number_of_taught_categories); 
				ROS_INFO("\t\t[-] *NOTE: the experiment is terminated because there is not enough test data to continue the evaluation");
				category_introduced.close();

				summary_of_experiment.open( evaluation_file.c_str(), std::ofstream::app);   
				summary_of_experiment << "\n-----------------------------------------------------------------------------------------------------------------------------------------";
				summary_of_experiment << " *NOTE: the experiment is terminated because there is not enough test data to continue the evaluation";
				summary_of_experiment << "\n-----------------------------------------------------------------------------------------------------------------------------------------";
				summary_of_experiment.close();
				
				ros::Duration duration = ros::Time::now() - begin_process;
			
				reportExperimentResult( average_class_precision,
										  number_of_instances, 
										  number_of_taught_categories,  
										  evaluation_file, duration);		    		    
				
				reportCurrentResults( TP, FP, FN, evaluation_file, true);
	
				reportAllExperimentalResults( TP, FP, FN, obj_num,			    
											  average_class_precision,
											  number_of_instances, 
											  number_of_taught_categories,
											  name_of_approach);
		
				monitorF1VsLearnedCategory( f1_vs_learned_category, TP, FP, FN);
				
				plotSimulatedTeacherProgressInMatlab( experiment_number, protocol_threshold, precision_file);
				//plotLocalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, protocol_threshold, local_f1_vs_learned_category.c_str());
				plotGlobalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, f1_vs_learned_category.c_str());
				plotNumberOfLearnedCategoriesVsIterationsInMatlab( experiment_number, category_introduced_txt.c_str());
				plotNumberOfStoredInstancesPerCategoryInMatlab( list_of_object_category);
				// system_command = "cp " + dataset_path + "/Category/Category.txt " + ros::package::getPath("rug_simulated_user") + "/result/experiment_1";
				// system( system_command.c_str());
				return (0) ;	    
			}
			else
			{
				std::string ground_truth_category_name = extractCategoryName(instance_path);
				instance_path = dataset_path + "/" + instance_path.c_str();
				
				//load an instance from file
				boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
				try 
				{ 
					io::loadPCDFile <PointXYZRGBA> (instance_path.c_str(), *target_pc) ;
				} 
				catch (const std::exception& e) 
				{ 
					ROS_ERROR("\t\t[-] could not read given object %s :", instance_path.c_str());					
					continue; 
				}		   
				ROS_INFO("\t\t[-] track_id: %i , \tview_id: %i ", track_id, view_id );
			
				/* _________________________________
				|                                   |
				|  compute orthographic projection  |
				|___________________________________| */

				boost::shared_ptr<pcl::PointCloud<PointT> > pca_object_view (new PointCloud<PointT>);
				boost::shared_ptr<PointCloud<PointT> > pca_pc (new PointCloud<PointT>); 
				vector < boost::shared_ptr<pcl::PointCloud<PointT> > > vector_of_projected_views;
				double largest_side = 0;
				int  sign = 1;
				vector <float> view_point_entropy;
				string list_of_sorted_projected_plane;
				Eigen::Vector3f center_of_bbox;
				vector< float > object_description;          
			
				computeDepthBasedGoodDescriptorAndSaveDepthAndRGBProjections( target_pc,
																			adaptive_support_lenght,
																			global_image_width,
																			threshold,
																			orthographic_image_resolution,
																			pca_object_view,
																			center_of_bbox,
																			vector_of_projected_views, 
																			largest_side, 
																			sign,
																			view_point_entropy,
																			list_of_sorted_projected_plane,
																			object_description ); 
					
				string view_with_max_enntropy = list_of_sorted_projected_plane;
				view_with_max_enntropy.resize(3);													
				ROS_INFO ("\t\t[-] list_of_sorted_projected_plane =  %s", list_of_sorted_projected_plane.c_str());
				ROS_INFO ("\t\t[-] view_with_max_enntropy = %s", view_with_max_enntropy.c_str());

				SITOV object_representation;
				for (size_t i = 0; i < object_description.size(); i++)
				{
					object_representation.spin_image.push_back(object_description.at(i));
				}

				ROS_INFO("\t\t[-] size of object view histogram %ld",object_representation.spin_image.size());
						
				cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
				cv_ptr->encoding = "bgr8";
				string img_name = "/tmp/"+ view_with_max_enntropy +"_RGB.jpg";
				cv_ptr->image = cv::imread(img_name.c_str(), CV_LOAD_IMAGE_COLOR);      
		
				cv_bridge::CvImagePtr cv_ptr_depth(new cv_bridge::CvImage);
				cv_ptr_depth->encoding = "bgr8";
				img_name = "/tmp/"+ view_with_max_enntropy +"_merged.jpg";
				cv_ptr_depth->image = cv::imread(img_name.c_str(), CV_LOAD_IMAGE_COLOR);      
				
				if(isImageEmpty(cv_ptr_depth))
					continue;

				category_introduced<< "0\n";

				SITOV deep_object_representation;
				/// call deep learning service to represent the given GOOD description as vgg16  
				rug_deep_feature_extraction::deep_representation srv;
				srv.request.good_representation = object_representation.spin_image;
				srv.request.RGB_image = *cv_ptr->toImageMsg();
				srv.request.depth_image = *cv_ptr_depth->toImageMsg();
				
				if (deep_learning_server.call(srv))
				{
					for (size_t i = 0; i < srv.response.deep_representation.size(); i++)
					{
						deep_object_representation.spin_image.push_back(srv.response.deep_representation.at(i));
					}
				}
				else
				{
					ROS_ERROR("\t\t[-] *ERROR: Failed to call deep learning service ");
				}

				ROS_INFO("\t\t[-] size of deep object representation is %d", deep_object_representation.spin_image.size());


				/* ______________________
				|                        |
				|   Object Recognition   |
				|________________________| */
			
				//// get list of all object categories
				list_of_object_category = _pdb->getAllObjectCat();
				while (list_of_object_category.size() == 0)
				{
					list_of_object_category = _pdb->getAllObjectCat();
					ros::Duration(0.2).sleep(); // sleep for 0.2 a second
				}	

				ROS_INFO("\t\t[-] %d categories exist in the perception database ", list_of_object_category.size() );
				
				ros::Time start_time_recognition = ros::Time::now();
				vector <float> object_category_distance;            
                vector <KNN_struct> KNN_all_categories; // used for KNN classifier

				for (size_t i = 0; i < list_of_object_category.size(); i++) // retrieves all categories from perceptual memory
				{
					if (list_of_object_category.at(i).rtov_keys.size() > 1) // check the size of category
					{    
						
						//ROS_INFO( "%s category has %d views",  list_of_object_category.at(i).cat_name.c_str(), list_of_object_category.at(i).rtov_keys.size());

						/* _____________________________________________
						|                                              |
						|   retrieves all instacnes of each category   |
						|______________________________________________| */
						
						std::vector<SITOV> category_instances; 
						for (size_t j = 0; j < list_of_object_category.at(i).rtov_keys.size(); j++) 
						{
							vector<SITOV> objectViewHistogram = _pdb->getSITOVs(list_of_object_category.at(i).rtov_keys.at(j).c_str());
							while (objectViewHistogram.size() == 0)
							{
								objectViewHistogram = _pdb->getSITOVs(list_of_object_category.at(i).rtov_keys.at(j).c_str());
								ros::Duration(0.2).sleep(); // sleep for 0.2 a second
							}
								
							if (objectViewHistogram.size() > 0)
								category_instances.push_back(objectViewHistogram.at(0));                      
						}
								
						/* ________________________________________________________________________________________
						|                                                                                          |
						|   Compute the dissimilarity between the target object and all instances of a category    |
						|__________________________________________________________________________________________| */
						
						
						float min_distance_object_category;
                        int best_matched_index;
                        float normalized_distance;
                        KNN_struct tmp_knn;
                        vector <float > distances;

						/// objectCategoryDistance() -> returns vector of distacnes, minimum distance and best_matched_index                     
						distances = objectCategoryDistance( deep_object_representation, 
															category_instances,
															min_distance_object_category, 
															best_matched_index, distance_function, pp); 		
									
						std::sort(distances.begin(), distances.end());                     
                        tmp_knn.distances = distances;
                        tmp_knn.category_name = list_of_object_category.at(i).cat_name.c_str();                    
                        KNN_all_categories.push_back(tmp_knn);

                        min_distance_object_category =  tmp_knn.distances.at(0);
						object_category_distance.push_back(min_distance_object_category);
					}
					else 
					{
						object_category_distance.push_back(1000000000);
					}
				}

				//// Object Recognition 

				std::string result_string;           
                float minimum_distance = 9000000000;
                knnClassification(K_for_KNN, KNN_all_categories, minimum_distance, result_string);

				ROS_INFO("\t\t[-] ========> Object recognition process took = %f", (ros::Time::now() - start_time_recognition).toSec());

				
				//// RRTOV stands for Recognition Result of Track Object View - it is a msg defined in race_perception_msgs
				RRTOV rrtov;
				rrtov.header.stamp = ros::Time::now();
				rrtov.track_id = track_id;
				rrtov.view_id = view_id;
				rrtov.recognition_result = result_string;
				rrtov.minimum_distance = minimum_distance;
				rrtov.ground_truth_name = instance_path.c_str();
				
				evaluationFunction(rrtov);
			
				
				if (c >= number_of_taught_categories)
				{
					c = 1;
				}
				else
				{
					c++;
				}

				
				if ( (iterations >= number_of_taught_categories) and (iterations <= window_size * number_of_taught_categories))
				{    
					if (( tp_tmp + fp_tmp) != 0)
					{
						precision = tp_tmp / double (tp_tmp+fp_tmp);
					}
					else
					{
						precision = 0;
					}		
					if ((tp_tmp + fn_tmp) != 0)
					{
						recall = tp_tmp / double (tp_tmp+fn_tmp);
					}
					else
					{
						recall = 0;
					}
					if ((precision + recall) != 0)
					{
						f1 = 2 * (precision * recall) / (precision + recall);
					}
					else
					{
						f1 = 0;
					}
						
					monitorPrecision (precision_file, f1);		

					if (f1 > protocol_threshold)
					{
						average_class_precision.push_back(f1);
						user_sees_no_improvement = false;
						ROS_INFO("\t\t[-] precision= %f", precision);
						ROS_INFO("\t\t[-] f1 = %f", f1); 
						reportCurrentResults(tp_tmp, fp_tmp, fn_tmp, evaluation_file, false);
						iterations = 1;
						monitorPrecision (local_f1_vs_learned_category, f1);
						ros::spinOnce();		
					}  
						
				}//if
				else if ( (iterations > window_size * number_of_taught_categories)) // In this condition, if we are at iteration I>3n, we only
																					// compute precision as the average of last 3n, and discart the first
																					// I-3n iterations.
				{
					//compute f1 of last 3n, and discart the first I-3n iterations
					f1 = computeF1OfLast3n (recognition_results, number_of_taught_categories);
					// ROS_INFO("\t\t[-]- precision= %f", precision);
					monitorPrecision (precision_file, f1);		
					reportF1OfLast3n (evaluation_file, f1);

					if (f1 > protocol_threshold)  
					{
						average_class_precision.push_back(f1);
						user_sees_no_improvement = false;
						reportCurrentResults(tp_tmp, fp_tmp, fn_tmp, evaluation_file, false);
						monitorPrecision (local_f1_vs_learned_category, f1);
						iterations = 1;
						iterations_user_sees_no_improvment = 0;
						ros::spinOnce();		
					} 
					else 
					{
						iterations_user_sees_no_improvment ++;
						ROS_INFO("\t\t[-] %i user_sees_no_improvement_in_f1", iterations_user_sees_no_improvment);

						if (iterations_user_sees_no_improvment > user_sees_no_improvment_const)
						{
							average_class_precision.push_back(f1);

							user_sees_no_improvement = true;
							ROS_INFO("\t\t[-] user_sees_no_improvement");
							ROS_INFO("\t\t[-] finish"); 
							ROS_INFO("\t\t[-] number of taught categories= %i", number_of_taught_categories); 
							
							summary_of_experiment.open (evaluation_file.c_str(), std::ofstream::app);
							summary_of_experiment << "\n After " << user_sees_no_improvment_const <<" iterations, user sees no improvement in precision";
							summary_of_experiment.close();

							monitorPrecision( local_f1_vs_learned_category, f1);
							monitorF1VsLearnedCategory( f1_vs_learned_category, TP, FP, FN );

							
							ros::Duration duration = ros::Time::now() - begin_process;
							reportExperimentResult( average_class_precision,
													  number_of_instances, 
													  number_of_taught_categories,  
													  evaluation_file, duration);
							category_introduced.close();

							reportCurrentResults( TP, FP, FN, evaluation_file, true);
							
							reportAllExperimentalResults( TP, FP, FN, obj_num,			    
															average_class_precision,
															number_of_instances, 
															number_of_taught_categories,
															name_of_approach);
							
							plotSimulatedTeacherProgressInMatlab( experiment_number, protocol_threshold, precision_file);
							// plotLocalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, protocol_threshold, local_f1_vs_learned_category.c_str());
							plotGlobalF1VsNumberOfLearnedCategoriesInMatlab( experiment_number, f1_vs_learned_category.c_str());
							plotNumberOfLearnedCategoriesVsIterationsInMatlab( experiment_number, category_introduced_txt.c_str());
							plotNumberOfStoredInstancesPerCategoryInMatlab( list_of_object_category);

							return 0 ;
						}
					}
				}
				else
				{
					float f1_system =0;
	
					if ((tp_tmp + fp_tmp)!=0)
					{
						precision = tp_tmp / double(tp_tmp+fp_tmp);
					}
					else
					{
						precision = 0;
					}		
					if ((tp_tmp+fn_tmp)!=0)
					{
						recall = tp_tmp / double(tp_tmp+fn_tmp);
					}
					else
					{
						recall = 0;
					}
					if ((precision + recall) != 0)
					{
						f1_system = 2 * (precision * recall) / (precision + recall);
					}
					else
					{
						f1_system = 0;
					}
					
					monitorPrecision( precision_file, f1_system);
				}
				k++; // k<-k+1 : number of classification result
				iterations	++;
			}//else	 
	    }//while
		monitorF1VsLearnedCategory (f1_vs_learned_category, TP, FP, FN );
	}
	
	ROS_INFO("\t\t[-] finish"); 
	ROS_INFO("\t\t[-] number of taught categories= %i", number_of_taught_categories); 

	//get toc
	ros::Duration duration = ros::Time::now() - begin_process;
	
	monitorPrecision( local_f1_vs_learned_category, f1);
	monitorF1VsLearnedCategory( f1_vs_learned_category, TP, FP, FN );

	reportCurrentResults( TP, FP, FN, evaluation_file, true);
	reportExperimentResult( average_class_precision,
							  number_of_instances, 
							  number_of_taught_categories,  
							  evaluation_file, duration);	
	
	category_introduced.close();
	reportAllExperimentalResults( TP, FP, FN, obj_num,			    
									average_class_precision,
									number_of_instances, 
									number_of_taught_categories,
									name_of_approach);
	
	plotSimulatedTeacherProgressInMatlab(experiment_number, protocol_threshold, precision_file);
	// plotLocalF1VsNumberOfLearnedCategoriesInMatlab (experiment_number, protocol_threshold, local_f1_vs_learned_category.c_str());
	plotGlobalF1VsNumberOfLearnedCategoriesInMatlab (experiment_number, f1_vs_learned_category.c_str());
	plotNumberOfLearnedCategoriesVsIterationsInMatlab (experiment_number, category_introduced_txt.c_str());
	plotNumberOfStoredInstancesPerCategoryInMatlab( list_of_object_category);
	    
    return 0 ;
}






				