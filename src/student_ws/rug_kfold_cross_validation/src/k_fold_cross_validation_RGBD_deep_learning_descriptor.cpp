// ############################################################################
//    
//   Created: 	2/07/2019
//   Author : 	Hamidreza Kasaei
//   Email  :	hamidreza.kasaei@rug.nl
//   Web    :   www.ai.rug.nl/hkasaei
//   Purpose:  This program provides a K-fold cross validation algorithm 
//             to evaluate an intasnce-based learning approach for 3D object 
//             recognition approaches in terms of precision and recall. 
//             In each iteration, a single fold is used for testing, and the 
//             remaining data are used as training data. The cross-validation 
//             process is then repeated 10 times, which each of the 10 folds 
//             used exactly once as the test data.  
// ############################################################################

/* _________________________________
   |                                 |
   |          RUN SYSTEM BY          |
   |_________________________________| */
   
//rm -rf /tmp/pdb
//roslaunch race_kfold_cross_validation_evaluation dictionary_based_K_fold_cross_validation.launch

/* _________________________________
  |                                 |
  |             INCLUDES            |
  |_________________________________| */
  
//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>

//ros includes 
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <pluginlib/class_list_macros.h> 
#include <stdio.h>
#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>

//pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <CGAL/Plane_3.h>
#include <pcl_conversions/pcl_conversions.h>

//package includes
#include <object_descriptor/object_descriptor_functionality.h>
#include <rug_kfold_cross_validation/rug_kfold_cross_validation_functionality.h>
#include <feature_extraction/spin_image.h>
#include <race_perception_msgs/perception_msgs.h>
#include <object_conceptualizer/object_conceptualization.h>
#include <race_perception_utils/print.h>
#include <race_perception_msgs/CompleteRTOV.h>
// 
#include <object_descriptor/object_descriptor_functionality.h>
#include <rug_deep_feature_extraction/deep_representation.h>


/* _________________________________
|                                 |
|            constant            |
|_________________________________| */
#define spin_image_width 8
#define subsample_spinimages 0
#define spin_image_support_lenght 0.05

/* _________________________________
   |                                 |
   |            NameSpace            |
   |_________________________________| */

using namespace std;
using namespace pcl;
using namespace ros;
using namespace race_perception_db;
using namespace race_perception_msgs;
using namespace race_perception_utils;

typedef pcl::PointXYZRGBA PointT;


PerceptionDB* _pdb;


/* __________________________
|                            |
|         Parameters         |
|____________________________| */

string name_of_approach = "cognitive_robotics_course"; 
string deep_learning_service = "/orthographicNet_service";
string base_network = "mobileNetV2";

string pooling_function = "MAX";
int  orthographic_image_resolution = 50; 
std::string distance_function = "chiSquared";
int K_for_KNN = 3;
bool running_a_bunch_of_experiments = false;
bool pdb_loaded = false;
std::string dataset_path = "";
double recognition_threshold;


///// do not change these params
int  adaptive_support_lenght = 1;
double global_image_width =0.5;
double downsampling_voxel_size = 0.001;
int threshold = 1000;	
double uniform_sampling_size = 0.05;
bool downsampling = false;


/* _______________________________
|                                 |
|         Global Variable         |
|_________________________________| */

std::string evaluation_file;
ofstream results , train_csv, test_csv;
int TP =0, FP=0, FN=0, TPtmp =0, FPtmp=0, FNtmp=0, obj_no=0;

std::vector< std::vector <int> > confusion_matrix;
std::vector<string> map_category_name_to_index;


void evaluationFunction(const race_perception_msgs::RRTOV &result)
{

    PrettyPrint pp;
    string tmp = result.ground_truth_name.substr(dataset_path.size(),result.ground_truth_name.size());
    string true_category = extractCategoryName(tmp);
    std:: string object_name;
    object_name = extractObjectName (result.ground_truth_name);
    pp.info(std::ostringstream().flush() << "[-]object_name: "<< object_name.c_str()); 
    
    obj_no++;
    pp.info(std::ostringstream().flush() << "[-]track_id="<<result.track_id << "\tview_id=" << result.view_id);
    
    //// print the minimum distance of the given object to each category
    // pp.info(std::ostringstream().flush() << "\n");
    // pp.info(std::ostringstream().flush() << "[-]object_category_distance:");
    // for (size_t i = 0; i < result.result.size(); i++)
    // {
	//     //pp.info(std::ostringstream().flush() << "-"<< result.result.at(i).normalized_distance);
    //     pp.info(std::ostringstream().flush() <<"\t D(target_object, "<<  result.result.at(i).cat_name.c_str() 
    //                                          <<") = "<< result.result.at(i).normalized_distance);
    // }
    // pp.info(std::ostringstream().flush() << "\n");
    
    if ( result.result.size() <= 0)
    {
	    pp.warn(std::ostringstream().flush() << "Warning: size of result is 0");
    }
       
    float minimum_distance = result.minimum_distance;

    string predict_category;
    predict_category= result.recognition_result.c_str();
    
    confusionMatrixGenerator(  true_category.c_str(), predict_category.c_str(), 
                               map_category_name_to_index,
                               confusion_matrix );


    pp.info(std::ostringstream().flush() << "[-]object_name: "<< object_name.c_str());
    pp.info(std::ostringstream().flush() << "[-]true_category: "<<true_category.c_str());
    pp.info(std::ostringstream().flush() << "[-]predict_category: " << predict_category.c_str());
    pp.info(std::ostringstream().flush() << "[-]minimum_distance: " << minimum_distance);


    char unknown[] = "unknown";
        
    results.open( evaluation_file.c_str(), std::ofstream::app);    
    results.precision(4);
    
    if ((strcmp(true_category.c_str(),unknown)==0) && (strcmp(predict_category.c_str(),unknown)==0))
    { 	
        results << "\n"<<obj_no<<"\t"<<object_name <<"\t\t\t"<< true_category <<"\t\t"<< predict_category <<"\t\t\t"<< "0\t0\t0"<< "\t\t"<< minimum_distance;
        results << "\n-----------------------------------------------------------------------------------------------------------------------------------";
    }
    else if ((strcmp(true_category.c_str(),predict_category.c_str())==0))
    { 
        TP++;
        TPtmp++;	
        results << "\n"<<obj_no<<"\t"<<object_name <<"\t\t\t"<< true_category <<"\t\t"<< predict_category <<"\t\t\t"<< "1\t0\t0" << "\t\t"<< minimum_distance;
        results << "\n-----------------------------------------------------------------------------------------------------------------------------------";
    }
    else if ((strcmp(true_category.c_str(),unknown)==0) && (strcmp(predict_category.c_str(),unknown)!=0))
    { 	
        FP++; 
        FPtmp++;
        results << "\n"<<obj_no<<"\t"<<object_name <<"\t\t\t"<< true_category <<"\t\t"<< predict_category <<"\t\t\t"<< "0\t1\t0"<< "\t\t"<< minimum_distance;
        results << "\n===================================================================================================================================";    }
    else if ((strcmp(true_category.c_str(),unknown)!=0) && (strcmp(predict_category.c_str(),unknown)==0))
    { 	
        FN++;
        FNtmp++;
        results << "\n"<<obj_no<<"\t"<<object_name <<"\t\t\t"<< true_category <<"\t\t"<< predict_category <<"\t\t\t"<< "0\t0\t1"<< "\t\t"<< minimum_distance;
        results << "\n===================================================================================================================================";
    }
    else if ((strcmp(true_category.c_str(),predict_category.c_str())!=0))
    {  	
        FP++; FN++;
        FPtmp++; FNtmp++;    
        results << "\n"<<obj_no<<"\t"<<object_name <<"\t\t\t"<< true_category <<"\t\t"<< predict_category <<"\t\t\t"<< "0\t1\t1"<< "\t\t"<< minimum_distance;
        results << "\n===================================================================================================================================";
    }
    results.close();

    pp.printCallback();
    ros::spinOnce();
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////

int kFoldTrainAndTestDataGenerator( int K_fold, 
                                    int iteration, 
                                    vector<vector<SITOV> > categories_instances,
                                    vector<vector<SITOV> > &categories_instances_train,
                                    vector<vector<SITOV> > &categories_instances_test)
{
    /// iteration [0-9]. K_fold = 10
    ROS_INFO(" ----------- fold%d -----------", iteration+1);
    for (size_t i = 0; i < categories_instances.size(); i++) // retrieves all categories 
    {                           
        
        int category_size = categories_instances.at(i).size();
        ROS_INFO("category_size = %d", category_size);

        int left_idx = 0, right_idx =0;

        left_idx = int(category_size/K_fold) * (iteration);
        
        ROS_INFO("left_idx = %d", left_idx);

        if (iteration != K_fold-1) 
        {
            right_idx = int(category_size/K_fold) * (iteration + 1);
        }
        else
        {
            right_idx = categories_instances.at(i).size();
        }
        ROS_INFO("right_idx = %d", right_idx);

        
        vector<SITOV> train_tmp;
        vector<SITOV> test_tmp;
        for (int j = 0; j < categories_instances.at(i).size(); j++)
        {
            if ((j >= left_idx) && (j < right_idx))   
            {
                test_tmp.push_back(categories_instances.at(i).at(j));
                // std::cout << "name test = " << categories_instances.at(i).at(j).ground_truth_name << endl; 
            }
            else
            {
                train_tmp.push_back(categories_instances.at(i).at(j));
                // std::cout << "name train = " << categories_instances.at(i).at(j).ground_truth_name << endl; 
            }
        }
        ROS_INFO("---------------------------------");

        categories_instances_train.push_back(train_tmp);
        categories_instances_test.push_back(test_tmp);
    
    }
        
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////


int conceptualizingAllDataUsingADeepTransferLearning( ros::ServiceClient deep_learning_server, 
                                                      string dataset_path,
                                                      int orthographic_image_resolution )
{

    int iteration=100; // should be a large number for this experiment
    int k_fold = 10;
    crossValidationDataCRC( k_fold, iteration, dataset_path);

    ROS_INFO("\n\n");
    ROS_INFO(" *****************************************************************");
    ROS_INFO(" **** Training process may take a few minutes, please wait!!! ****");
    ROS_INFO(" *****************************************************************");

    PrettyPrint pp;
    int track_id = 1;

    conceptualizingDeepLearningBasedOnRGBAndOrthographicImages( track_id, 
                                                                dataset_path,                                                                
                                                                deep_learning_server,
                                                                orthographic_image_resolution,
                                                                pp);

    return 0;
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    ros::init (argc, argv, "EVALUATION");
    ros::NodeHandle nh;
    string name = nh.getNamespace();
    bool map_name_to_idx_flag = false;

    /* ______________________________
    |                                |
    |       initialize database      |
    |________________________________| */
    _pdb = race_perception_db::PerceptionDB::getPerceptionDB(&nh); //initialize the database class_list_macros

    PrettyPrint pp;
    ros::Time beginProc = ros::Time::now(); //start tic	

    //create a folder to save all relevant the resuts
    string systemStringCommand= "mkdir "+ ros::package::getPath("rug_kfold_cross_validation")+ "/result/experiment_1";
    system( systemStringCommand.c_str());
    
    /* _____________________________________
    |                                       |
    |    read prameters from launch file    |
    |_______________________________________| */

    // read name_of_approach parameter
    nh.param<std::string>("/perception/name_of_approach", name_of_approach, name_of_approach);
    
    // read database parameter
    nh.param<std::string>("/perception/dataset_path", dataset_path, dataset_path);
    ROS_INFO("dataset_path = %s", dataset_path.c_str());

    // read deep_learning_service parameter
    nh.param<std::string>("/perception/deep_learning_service", deep_learning_service, deep_learning_service);

    // read base_network parameter
    nh.param<std::string>("/perception/base_network", base_network, base_network);


    // resolution of orthographic images [n x n]
    nh.param<int>("/perception/orthographic_image_resolution", orthographic_image_resolution, orthographic_image_resolution);

    // read pooling function  ["MAX", "AVG", "APP"] 
    nh.param<string>("/perception/pooling_function", pooling_function, pooling_function);

    // read distance_function parameter    
    nh.param<std::string>("/perception/distance_function", distance_function, "default_param");

    //K param for KNN
    nh.param<int>("/perception/K_for_KNN", K_for_KNN, K_for_KNN);

    // recognition threshold
    nh.param<double>("/perception/recognition_threshold", recognition_threshold, recognition_threshold);

    // read pdb_loaded parameter 0 = FLASE, 1 = TRUE 
    nh.param<bool>("/perception/pdb_loaded", pdb_loaded, pdb_loaded);
    string pdb_loaded_flag = (pdb_loaded == 0) ? "FALSE" : "TRUE";
    
    // read pdb_loaded parameter 0 = FLASE, 1 = TRUE 
    nh.param<bool>("/perception/running_a_bunch_of_experiments", running_a_bunch_of_experiments, running_a_bunch_of_experiments);
    string running_a_bunch_of_experiments_flag = (running_a_bunch_of_experiments == 0) ? "FALSE" : "TRUE";


    ///// do not change these params
    /// internal params of GOOD descriptor 
    nh.param<double>("/perception/global_image_width", global_image_width, global_image_width);		
    nh.param<int>("/perception/adaptive_support_lenght", adaptive_support_lenght, adaptive_support_lenght);
    nh.param<int>("/perception/threshold", threshold, threshold);
    
    // read downsampling parameter 0 = FLASE, 1 = TRUE 
    nh.param<bool>("/perception/downsampling", downsampling, downsampling);
    string downsampling_flag = (downsampling == 0) ? "FALSE" : "TRUE";

    // read downsampling_voxel_size parameter
    nh.param<double>("/perception/downsampling_voxel_size", downsampling_voxel_size, downsampling_voxel_size);
    downsampling_voxel_size = downsampling_voxel_size ;

    // read uniform_sampling_size parameter which is used for partial object view generation 
    nh.param<double>("/perception/uniform_sampling_size", uniform_sampling_size, uniform_sampling_size);

    
    pp.info(std::ostringstream().flush() <<"deep_learning based k-fold cross-validation -> Hello World");
    
    /* ______________________________________
    |                                       |
    |  Define path of train and test data   |
    |_______________________________________| */  
    
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string test_data_path = package_path + "/CV_test_instances.txt";
    string train_data_path = package_path + "/CV_train_instances.txt";
    
    /* _____________________________________
    |                                       |
    |     create a deeep learning server    |
    |_______________________________________| */
    ros::ServiceClient deep_learning_server = nh.serviceClient<rug_deep_feature_extraction::deep_representation>(deep_learning_service);

	int exp_num = 0;
	
    /* _____________________________________________________
    |							                            |  
    | expriments for varing Gussian Noise and downSampling  |
    |_______________________________________________________| */  

    // number_of_performed_experiments(name_of_approach, exp_num);
    string dataset= (dataset_path.find("restaurant_object_dataset") != std::string::npos) ? "Restaurant RGB-D Object Dataset" : "ModelNet10 Dataset";
   
    bool modelnet_dataset = (dataset_path.find("restaurant_object_dataset") != std::string::npos) ? false : true;
    string modelnet_dataset_flag = (modelnet_dataset == 0) ? "FALSE" : "TRUE";

    evaluation_file = ros::package::getPath("rug_kfold_cross_validation")+ "/result/experiment_1/summary_of_experiment.txt";
    results.open (evaluation_file.c_str(), std::ofstream::trunc);

    results <<"System configuration:" 
            << "\n\t-experiment_name = " << name_of_approach
            << "\n\t-name_of_dataset = " << dataset 
            << "\n\t-number_of_category = "<< "10" ////for both Restaurant RGB-D Object Dataset and ModelNet10
            << "\n\t-orthographic_image_resolution = "<< orthographic_image_resolution
            << "\n\t-deep_learning_service = " << deep_learning_service
            << "\n\t-base_network = " << base_network            
            << "\n\t-pooling function [MAX, AVG, APP] = " << pooling_function
            << "\n\t-distance_function = "<< distance_function              
            << "\n\t-pdb_loaded = "<< pdb_loaded_flag             
            << "\n\t-K param for KNN [1, 3, 5, ...] = " << K_for_KNN
            << "\n\t-running_a_bunch_of_experiments = " << running_a_bunch_of_experiments_flag
            << "\n\t-other parameters = " << "if you need to define more params, please add them here.";

    results.close();
    
    ros::Time begin_exp = ros::Time::now(); //start tic	

    double training_time;
    ros::Time begin_proc = ros::Time::now(); //start tic

    bool debug = false;

    /* ______________________
    |                       |
    |  Run train and test   |
    |_______________________| */ 

    int obj_no = 1;

    //// if modelnet dataset set k = 1, else k = 10
    size_t k_fold = 10; /// param

    /* __________________________
    |                           |
    |  conceptualize all data   |
    |___________________________| */

    int track_id = 1;
    int view_id = 1;

    if (!pdb_loaded)
    {
        ROS_INFO("\t\t[-]conceptualizing training data");  
        
        conceptualizingAllDataUsingADeepTransferLearning( deep_learning_server, 
                                                          dataset_path,
                                                          orthographic_image_resolution);

	//// get toc
        ros::Duration duration = ros::Time::now() - beginProc;
        training_time = duration.toSec();
        ros::param::set("/training_time", training_time);
        ROS_INFO("\t\t[-] training_time was %f seconds", training_time);  

        begin_proc = ros::Time::now(); //start test tic	
    }
    else
    {
        ros::param::get("/training_time", training_time);
        ROS_INFO("\t\t[-] training data has been loaded");  
        ROS_INFO("\t\t[-] training_time was %f seconds", training_time);  
    }    

    //// get list of all object categories
    vector <ObjectCategory> list_of_object_category = _pdb->getAllObjectCat();
    ROS_INFO(" %d categories exist in the perception database ", list_of_object_category.size() );

    //// initialize confusion_matrix
    if (!map_name_to_idx_flag)
    {  
        std::vector<int>  row (list_of_object_category.size(), 0);
        for (size_t i = 0; i < list_of_object_category.size(); i++ )
        {
            confusion_matrix.push_back (row);
            map_category_name_to_index.push_back(list_of_object_category.at(i).cat_name.c_str());
            map_name_to_idx_flag = true;
        }
    }

    ROS_INFO(" confusion_matrix has been initialized!" );


    /* ___________________________
    |                             |
    |   retrieves all data once   |
    |_____________________________| */
    
    ////TODO: can be a function named loadTrainingData(.)
    vector<vector<SITOV> > categories_instances; 
    vector<string> categories_name; 
    int number_of_objects = 0;
    for (size_t i = 0; i < list_of_object_category.size(); i++) // retrieves all categories from perceptual memory
    {
        if (list_of_object_category.at(i).rtov_keys.size() > 1) // check the size of category
        {                                    
            ////////////// retrieves all instacnes of each category //////////////
            vector<SITOV> instances_of_the_category; 

            ROS_INFO(" %s category has %d instances ", list_of_object_category.at(i).cat_name.c_str(), list_of_object_category.at(i).rtov_keys.size() );
            number_of_objects += list_of_object_category.at(i).rtov_keys.size();

            for (size_t j = 0; j < list_of_object_category.at(i).rtov_keys.size(); j++) 
            {
                vector<SITOV> objects_views_histograms = _pdb->getSITOVs(list_of_object_category.at(i).rtov_keys.at(j).c_str());
            
                if (objects_views_histograms.size() > 0)
                {
                    SITOV object_representation;
                    for (int k = 0; k < objects_views_histograms.at(0).spin_image.size(); k++)
                    {
                        object_representation.spin_image.push_back(objects_views_histograms.at(0).spin_image.at(k));
                    }
                    object_representation.ground_truth_name = objects_views_histograms.at(0).ground_truth_name;             
                    instances_of_the_category.push_back(object_representation);
                }							
            }            
            categories_name.push_back(list_of_object_category.at(i).cat_name.c_str());
            categories_instances.push_back (instances_of_the_category); 
            ROS_INFO("\t\t[-]size of %s category is = %d", list_of_object_category.at(i).cat_name.c_str(), 
                                                            categories_instances.at(i).size());          

        }
    }

    double average_recognition_time = 0;
    for (size_t iteration = 0; iteration < k_fold; iteration ++)
    {
        results.open (evaluation_file.c_str(), std::ofstream::app);
        results << "\n\nNo."<<"\tobject_name" <<"\t\t\t\t"<< "ground_truth" <<"\t"<< "prediction"<< "\t\t"<< "TP" << "\t"<< "FP"<< "\t"<< "FN \t\tdistance";
        results << "\n------------------------------------------------------------------------------------------------------------------------------------";
        results.close();


        vector<vector<SITOV> > categories_instances_train;
        vector<vector<SITOV> > categories_instances_test;

        kFoldTrainAndTestDataGenerator( k_fold, 
                                        iteration, 
                                        categories_instances,
                                        categories_instances_train,
                                        categories_instances_test);

        /* ______________________
        |                        |
        |   Object Recognition   |
        |________________________| */

        for (size_t cat_idx = 0; cat_idx < list_of_object_category.size(); cat_idx++) // retrieves all categories from perceptual memory
        {             
            for (size_t test_idx = 0; test_idx < categories_instances_test.at(cat_idx).size(); test_idx++) // retrieves all categories from perceptual memory
            {                                          
                SITOV deep_object_representation = categories_instances_test.at(cat_idx).at(test_idx);

                vector <NOCD> nocd_object_category_distance;
                vector <float> object_category_distance;       
                vector <KNN_struct> KNN_all_categories; // used for KNN classifier
                ros::Time start_time_recognition = ros::Time::now();

                for (size_t i = 0; i < list_of_object_category.size(); i++) // iterate over all categories
                {                           
                    if (categories_instances_train.at(i).size() > 1) // check the size of category
                    {                    
                        /* ________________________________________________________________________________________
                        |                                                                                          |
                        |   Compute the dissimilarity between the target object and all instances of a category    |
                        |__________________________________________________________________________________________| */
                        
                        ROS_INFO("size of training data for the %s category =  %d", categories_name.at(i).c_str(), categories_instances_train.at(i).size() );

                        float min_distance_object_category;
                        int best_matched_index;
                        float normalized_distance;
                        KNN_struct tmp_knn;
                        vector <float > distances;

                        /// objectCategoryDistance() -> returns vector of distacnes, minimum distance and best_matched_index                     
                        distances = objectCategoryDistance( deep_object_representation, 
                                                            categories_instances_train.at(i),
                                                            min_distance_object_category, 
                                                            best_matched_index, distance_function, pp);  

                        std::sort(distances.begin(), distances.end());                     
                        tmp_knn.distances = distances;
                        tmp_knn.category_name = categories_name.at(i);                    
                        KNN_all_categories.push_back(tmp_knn);
                        
                        min_distance_object_category =  tmp_knn.distances.at(0);
                        object_category_distance.push_back(min_distance_object_category);

                    }
                    else 
                    {
                        object_category_distance.push_back(1000000000);
                    }
                }

                std::string result_string;           
                float minimum_distance = 9000000000;
                knnClassification(K_for_KNN, KNN_all_categories, minimum_distance, result_string);
                
                average_recognition_time += (ros::Time::now() - start_time_recognition).toSec();
                ROS_INFO("\t\t[-]Object Recognition process took %f secs", (ros::Time::now() - start_time_recognition).toSec());

                //// RRTOV stands for Recognition results of Track Object View - it is a msg defined in race_perception_msgs
                RRTOV rrtov;
                rrtov.header.stamp = ros::Time::now();
                rrtov.track_id = track_id;
                rrtov.view_id = view_id;
                rrtov.recognition_result = result_string;
                rrtov.minimum_distance = minimum_distance;
                rrtov.ground_truth_name = deep_object_representation.ground_truth_name;
                //rrtov.result = nocd_object_category_distance;

                evaluationFunction(rrtov);
            
                ros::spinOnce();
                track_id ++;
            }
        }

        //list_of_test_data.close();
        reportCurrentResults(TPtmp, FPtmp, FNtmp, evaluation_file,0);
        ros::spinOnce();

    }


    results.close();
    
    //// get toc
    ros::Duration test_time = ros::Time::now() - beginProc;
    double duration_sec = training_time + test_time.toSec();
    
    reportCurrentResults(TP, FP, FN, evaluation_file,1);
    results.open (evaluation_file.c_str(), std::ofstream::app);
    results << "\n\t - This expriment took " << duration_sec << " secs, and object recognition on average took "<< average_recognition_time / (float) number_of_objects <<" secs\n\n";
    results << "========================================================================== \n"; 


    results << "\n\n - Confusion_matrix [" << confusion_matrix.size() << "," << confusion_matrix.at(0).size() << "]= \n\n";
    
    double avg_class_accuracy = 0;
    for (int i = 0; i < confusion_matrix.size(); i++ ) 
    {
        int sum_all_predicitions = 0;
        results << map_category_name_to_index.at(i) << ",\t\t";
        for (int j = 0; j < confusion_matrix.at(i).size(); j++ ) 
        {
            sum_all_predicitions += confusion_matrix.at(i).at(j);
            results << confusion_matrix.at(i).at(j) << ",\t";  
        }
        // ROS_INFO ("confusion_matrix.at(i).at(i) = %d", confusion_matrix.at(i).at(i));
        // ROS_INFO ("sum_all_predicitions = %d", sum_all_predicitions);

        float class_accuracy_tmp = float (confusion_matrix.at(i).at(i)) / float(sum_all_predicitions);
        results << "\t" << map_category_name_to_index.at(i) << " accuracy = " << class_accuracy_tmp;    
        results << "\n";
        avg_class_accuracy += class_accuracy_tmp;
    }

    results << "\t\t";
    for (int i =0; i < confusion_matrix.size(); i++ ) 
    {
        results << map_category_name_to_index.at(i) << ", ";
    }
    
    avg_class_accuracy = avg_class_accuracy / float (confusion_matrix.size());
    results << "\n\t - global_class_accuracy = " << avg_class_accuracy;
    results.close();


    reportAllDeepTransferLearningExperiments (  TP, FP, FN,
											    "Restaurant",
											    orthographic_image_resolution, 
												base_network,											
												pooling_function,
												distance_function, 
												K_for_KNN,
												avg_class_accuracy, 
												name_of_approach, 
												duration_sec);

    plotConfusionMatrix(confusion_matrix, map_category_name_to_index, name_of_approach);

    ROS_INFO ("\t\t[-] experiment finished successfully!");

    ros::spinOnce();
    
    return 1;
}