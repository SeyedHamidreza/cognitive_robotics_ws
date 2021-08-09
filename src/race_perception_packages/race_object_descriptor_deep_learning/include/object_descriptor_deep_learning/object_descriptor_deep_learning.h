#ifndef _OBJECT_DESCRIPTOR_DEEP_LEARNING_H_
#define _OBJECT_DESCRIPTOR_DEEP_LEARNING_H_


//roslaunch race_feature_extraction test_pdbnodelet_node.launch
//Emulate race_object_tracking by : "rosrun race_feature_extraction test_feature_extraction"

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

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
#include <ros/package.h>

//package includes
#include <object_descriptor_deep_learning/object_descriptor_deep_learning_functionality.h>
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

//#include <race_deep_learning_feature_extraction/vgg16_model.h>
#include <race_deep_learning_feature_extraction/deep_representation.h>



/* _________________________________
   |                                 |
   |        Class definition         |
   |_________________________________| */

namespace ros {class Publisher;}

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
using namespace tf;


 /* _________________________________
  |                                 |
  |        Global Parameters        |
  |_________________________________| */

    //spin images parameters
    int    spin_image_width = 8 ;
    double spin_image_support_lenght = 0.1;
    int    subsample_spinimages = 10;
    double uniform_sampling_size = 0.06;

    int  off_line_flag  = 1;
    int  number_of_bins = 5;
    int  adaptive_support_lenght = 0;
    double global_image_width =0.5;
    
    int deep_learning = 0;

	std::string deep_learning_architecture = "/mobilenetV2_service";
	bool downsampling = false;
	bool image_normalization = true;
	bool multiviews = true;
	bool max_pooling = true;

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */
            
//
// typedef pcl::PointXYZRGB PointT;
typedef PointCloud<PointT> PointCloudT;
typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;



  
bool signDisambiguationFlag = false;
int sign = 1;
int threshold = 150;	 
boost::shared_ptr<tf::TransformListener> _p_transform_listener;


PointCloudPtrT cloud_reference;
PointCloudPtrT initial_cloud_ref;

boost::shared_ptr<TransformBroadcaster> _br; //a transform broadcaster


int AddingGaussianNoise (boost::shared_ptr<PointCloud<PointT> > input_pc, 
			 double standard_deviation,
			 boost::shared_ptr<PointCloud<PointT> > output_pc)

{
    ROS_INFO ("Adding Gaussian noise with mean 0.0 and standard deviation %f\n", standard_deviation);

    *output_pc = *input_pc;

    boost::mt19937 rng; rng.seed (static_cast<unsigned int> (time (0)));
    boost::normal_distribution<> nd (0, standard_deviation);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

    for (size_t point_i = 0; point_i < input_pc->points.size (); ++point_i)
    {
        output_pc->points[point_i].x = input_pc->points[point_i].x + static_cast<float> (var_nor ());
        output_pc->points[point_i].y = input_pc->points[point_i].y + static_cast<float> (var_nor ());
        output_pc->points[point_i].z = input_pc->points[point_i].z + static_cast<float> (var_nor ());
    }

    return 0;
 
}
int downSampling ( boost::shared_ptr<PointCloud<PointT> > cloud, 		
		  float downsampling_voxel_size, 
		  boost::shared_ptr<PointCloud<PointT> > downsampled_pc)
{
  
	// Downsample the input point cloud using downsampling voxel size
	PointCloud<int> sampled_indices;
	UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud (cloud);
	uniform_sampling.setRadiusSearch (downsampling_voxel_size/*0.01f*/);
	//uniform_sampling.compute (sampled_indices);////PCL 1.7;
	//copyPointCloud (*cloud, sampled_indices.points, *downsampled_pc);////PCL 1.7;

	uniform_sampling.filter (*downsampled_pc);//PCL 1.8;
	
  return 0;
}


namespace race_object_descriptor_deep_learning
{
    template <class PointT>
        class ObjectDescriptorDeepLearning: public nodelet::Nodelet 
    {
        public:
            //Type defs

            //local variables
            string _name;
            bool _verb;
            ros::NodeHandle* _p_nh; // The pointer to the node handle
            ros::NodeHandle _nh; // The node handle
            ros::NodeHandle _priv_nh; // The node handle
            ros::NodeHandle* _p_priv_nh; // The node handle
            bool _flg_is_nodelet; //a flag to check if this code is running as a node or a nodelet
            boost::shared_ptr<ros::Subscriber> _p_pcin_subscriber;
            boost::shared_ptr<ros::Subscriber> _p_pcin_subscriber2;

            //boost::shared_ptr<ros::Publisher> _p_pcin_publisher;
            boost::shared_ptr<ros::Publisher> _p_crtov_publisher;
            boost::shared_ptr<ros::Publisher> _p_projected_object_point_cloud_to_table_publisher;
            boost::shared_ptr<ros::Publisher> _p_projected_object_point_cloud_to_table_publisher2;

            boost::shared_ptr<class_colormap_utils> cm; //a colormap class
            
            Publisher marker_publisher;
            Publisher neat_marker_publisher;

            std::string _id_name;
            PerceptionDB* _pdb; //initialize the class
            boost::shared_ptr<CycleDebug> cd;

            tf::StampedTransform stf;
            tf::StampedTransform transf_object;

            ros::ServiceClient deep_learning_server;

            /* _________________________________
               |                                 |
               |           PARAMETERS			|
               |_________________________________| */

            //double _param1;
            //string _param2;

            /* _________________________________
               |                                 |
               |           CONSTRUCTORS          |
               |_________________________________| */

            ObjectDescriptorDeepLearning(){_flg_is_nodelet = true;};

            ObjectDescriptorDeepLearning(ros::NodeHandle* n)
            {
                _flg_is_nodelet = false; 
                _p_nh = n; //if this is a node set both the nodehandle and private node handle to n
                _p_priv_nh = n; 
                onInit();
            };

            /**
             * @brief Destructor
             */
            ~ObjectDescriptorDeepLearning()
            {
                PrettyPrint pp(_name);
                pp.info(std::ostringstream().flush() << _name.c_str() << ": Destructor called");
                pp.info(std::ostringstream().flush() << _name.c_str() << ": Finished destructor");
                pp.printCallback();


            };

	          /* _________________________________
               |                                 |
               |           CLASS METHODS         |
               |_________________________________| */

             /* _________________________________
               |                                 |
               |           CLASS METHODS         |
               |_________________________________| */

            void onInit(void)
            {
                //create a node handle in internal nh_ variable, and point p_nh_
                //to it. Only done if we are using a nodelet.
                if (_flg_is_nodelet == true)
                {
                    _nh = getNodeHandle(); 
                    _p_nh = &_nh;
                    _priv_nh = getPrivateNodeHandle(); 
                    _p_priv_nh = &_priv_nh;
                }

                /////////////////////////////////
                /* ______________________________
                   |                             |
                   |  working area for grasping  |
                   |_____________________________| */
                
                //init listener
                _p_transform_listener = (boost::shared_ptr<tf::TransformListener>) new tf::TransformListener;
                ros::Duration(0.5).sleep();
                ROS_INFO(" a tf lisener has been created" )  ;	
                //init broadcaster
                _br = (boost::shared_ptr<TransformBroadcaster>) new TransformBroadcaster;

                ROS_INFO(" a tf broadcaster has been created" )  ;	

                _id_name = "_ObjID_";
                //Initialize tf stuff

                //initialize parameters
                _name = _p_priv_nh->getNamespace();

                PrettyPrint pp(_name);

                _p_priv_nh->param<bool>("verbose", _verb , false);
                _p_priv_nh->param<int>("verbose", deep_learning , deep_learning);


                //read spin images parameters
                // _p_priv_nh->param<int>("/perception/spin_image_width", spin_image_width, spin_image_width);
                // _p_priv_nh->param<double>("/perception/spin_image_support_lenght", spin_image_support_lenght, spin_image_support_lenght);
                // _p_priv_nh->param<int>("/perception/subsample_spinimages", subsample_spinimages, subsample_spinimages);
                // _p_priv_nh->param<double>("/perception/uniform_sampling_size", uniform_sampling_size, uniform_sampling_size);

                //new object descriptor parameters
                _p_priv_nh->param<int>("/perception/number_of_bins", number_of_bins, number_of_bins);
                _p_priv_nh->param<double>("/perception/global_image_width", global_image_width, global_image_width);
                _p_priv_nh->param<int>("/perception/adaptive_support_lenght", adaptive_support_lenght, adaptive_support_lenght);

                signDisambiguationFlag = false;

                //Create a new colormap	
                cm = (boost::shared_ptr<class_colormap_utils>) new class_colormap_utils(std::string("hot"),5, 1, false);

            	//create a cycle debug
                cd = (boost::shared_ptr<CycleDebug>) new CycleDebug(_p_nh, _name);

                //initialize the subscriber
                //topic name is computed by getting the namespace path and then adding
                unsigned found = _name.find_last_of("/\\");
                std::string pcin_topic = _name.substr(0,found) + "/tracker/tracked_object_point_cloud";
                ROS_INFO("pcin");
                _p_pcin_subscriber = (boost::shared_ptr<ros::Subscriber>) new ros::Subscriber;
                *_p_pcin_subscriber = _p_nh->subscribe (pcin_topic, 1, &ObjectDescriptorDeepLearning::callback, this);

                //initialize the Publisher
                _p_crtov_publisher = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
                *_p_crtov_publisher = _p_priv_nh->advertise<race_perception_msgs::CompleteRTOV> ("histogram_tracked_object_view", 1000);

                //initialize the publishers
                neat_marker_publisher = _p_nh->advertise<visualization_msgs::MarkerArray>("/perception/object_descriptor/neat_markers", 100);


                _p_crtov_publisher = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
                *_p_crtov_publisher = _p_priv_nh->advertise<race_perception_msgs::CompleteRTOV> ("new_histogram_tracked_object_view", 100);

                _p_projected_object_point_cloud_to_table_publisher = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
                *_p_projected_object_point_cloud_to_table_publisher = _p_nh->advertise<sensor_msgs::PointCloud2>("/projected_object_point_clouds", 100);

                _p_projected_object_point_cloud_to_table_publisher2 = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
                *_p_projected_object_point_cloud_to_table_publisher2 = _p_nh->advertise<sensor_msgs::PointCloud2>("/projected_object_point_clouds_after_transform", 100);

    
                //initialize the database
                _pdb = race_perception_db::PerceptionDB::getPerceptionDB(_p_priv_nh, _flg_is_nodelet); //initialize the database class
                
				//initialize a server for deep learning based representation 
				///TODO: this should be a parameter
				//The following network architectures have been implemeneted and can be used: 
				// "/vgg16_service", "/vgg19_service", "/xception_service", "/resnet50_service", "/mobilenet_service", ...  
				// ..., "/densenet121_server", "/densenet169_server", "/densenet201_server", "/nasnet_large_server", "/nasnet_mobile_server", "/mobilenetV2_service", ... 
				// ..., "/inception_resnet_service",   "/inception_service"
                
				// read deep_learning_architecture parameter
			    _p_priv_nh->param<std::string>("/perception/deep_learning_architecture", deep_learning_architecture, deep_learning_architecture);

				// read image_normalization parameter 0 = FLASE, 1 = TRUE 
				_p_priv_nh->param<bool>("/perception/image_normalization", image_normalization, image_normalization);
				string image_normalization_flag = (image_normalization == 0) ? "FALSE" : "TRUE";

				// read multiviews parameter 0 = FLASE, 1 = TRUE 
				_p_priv_nh->param<bool>("/perception/multiviews", multiviews, multiviews);
				string multiviews_flag = (multiviews == 0) ? "FALSE" : "TRUE";

				// read max_pooling parameter 0 = FLASE, 1 = TRUE 
				_p_priv_nh->param<bool>("/perception/max_pooling", max_pooling, max_pooling);
				string pooling_flag = (max_pooling == 0) ? "Avg" : "Max";

							
				// read downsampling parameter 0 = FLASE, 1 = TRUE 
				_p_priv_nh->param<bool>("/perception/downsampling", downsampling, downsampling);
				string downsampling_flag = (downsampling == 0) ? "FALSE" : "TRUE";

				// // read downsampling_voxel_size parameter
				// nh.param<double>("/perception/downsampling_voxel_size", downsampling_voxel_size, downsampling_voxel_size);
				// downsampling_voxel_size = downsampling_voxel_size * 0.02;

				
                // deep_learning_server = _p_priv_nh->serviceClient<race_deep_learning_feature_extraction::deep_representation>("/mobilenetV2_service");
				deep_learning_server = _p_priv_nh->serviceClient<race_deep_learning_feature_extraction::deep_representation>(deep_learning_architecture);
    
                //Output initialization information
                pp.printInitialization();				

            };

	    
            /**
             * @brief This is the callback.
             * @param msg
             */
            void callback(const race_perception_msgs::PCTOV::ConstPtr& msg)
            {
					//NOTE : there is a bug in global one 
					
					//std::reverse(myvector.begin(),myvector.end());
					cd->tic();//cycle debug tic
					PrettyPrint pp(_name);
					ROS_INFO("//////////////////////////////// TID = %i ///////////////////////////////////////////", msg-> track_id);


					ROS_INFO("//////////////////////////////// dimensions (%2.2f, %2.2f, %2.2f)  ///////////////////////////////////////////", 
					msg-> dimensions.x , msg-> dimensions.y, msg-> dimensions.z	);
					ROS_INFO("//////////////////////////////// pose of the object = %2.2f, %2.2f, %2.2f ///////////////////////////////////////////", 
					msg-> pose_stamped.pose.position.x, msg-> pose_stamped.pose.position.y, msg-> pose_stamped.pose.position.z	);

					//read spin images parameters
					_p_priv_nh->param<int>("/perception/spin_image_width", spin_image_width, spin_image_width);
					_p_priv_nh->param<double>("/perception/spin_image_support_lenght", spin_image_support_lenght, spin_image_support_lenght);
					_p_priv_nh->param<int>("/perception/subsample_spinimages", subsample_spinimages, subsample_spinimages);
						_p_priv_nh->param<double>("/perception/uniform_sampling_size", uniform_sampling_size, uniform_sampling_size);

					_p_priv_nh->param<int>("deep_learning", deep_learning, deep_learning);
					pp.info(std::ostringstream().flush()<<"\t\t[-] deep_learning_flag :"<< deep_learning);

					//parameters of new object descriptor
					_p_priv_nh->param<int>("/perception/number_of_bins", number_of_bins, number_of_bins);
					_p_priv_nh->param<double>("/perception/global_image_width", global_image_width, global_image_width);		
					_p_priv_nh->param<int>("/perception/adaptive_support_lenght", adaptive_support_lenght, adaptive_support_lenght);
					_p_priv_nh->param<int>("/perception/sign", sign, sign);
					_p_priv_nh->param<int>("/perception/off_line_flag", off_line_flag, off_line_flag);
					//TODO threshold must be a parametes 
					
					pp.info(std::ostringstream().flush()<<"\t\t[-] spin_image_width :"<< spin_image_width);
					pp.info(std::ostringstream().flush()<<"\t\t[-] spin_image_support_lenght :"<< spin_image_support_lenght);
					pp.info(std::ostringstream().flush()<<"\t\t[-] subsample_spinimages :"<< subsample_spinimages);
					
					//initializ 3 plane histogram for the given object
									
					ros::Time beginProc = ros::Time::now(); //start tic
					
					//Declare a boost share ptr to the pointCloud
					boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>); 
					pcl::fromROSMsg(msg->point_cloud,*target_pc ); //Convert the pointcloud msg to a pcl point cloud
					
					ROS_INFO("given point cloud has %d points ", target_pc->points.size());
					
				
					//NOTE : Rotation matrix can be formed in several ways. 
					// One way is to get it from rotation axis and rotation angle:
					// 
					// 1 - Get the plane coefficients of your ground plane from RANSAC. First three coefficients correspond to your ground planes normal (table). 
					// 2 - Generate the normal vector for your desired plane. If it is xy plane, since in xy plane z is 0, its normal is x=0,y=0 and z=1. 
					// 3 - Calculate cross product of normal of ground plane and normal of xy plane to get rotation vector (axis) which is a unit vector. 
					// 4 - Calculate the rotation angle. Angle between the planes is equal to angle between the normals. From the definition of the dot product, 
					//     you can extract the angle between two normal vectors. In case of XY plane, it is equal to theta=arccos(C/sqrt(A^2+B^2+C^2) where A, B, C are first three coefficients of ground plane. 
					// 5 - You now have rotation vector and rotation angle. You can generate the rotation matrix (3x3) or quaternion. Look for the formula in Wikipedia. 
					// 6 - Generate the complete transformation matrix. If you do not perform any translation or scaling, your third row and column will have zero except (4,4) entry which is 1. 
					// 7 - Apply the transformation simply with transformPointCloud(cloud,transformed,transformationMatrix) 
							
					/* ______________________________________________________________
                   |                     				       	    |
                   |  compute projection of the given object to three main axes   |
                   |______________________________________________________________| */

					string _fixed_frame_id_tmp = "/odom_combined";
					string _table_frame_id = "/perception/tabletop_segmentation/table";
					
					std::string tracker_frame_id = "/perception/pipeline" + boost::lexical_cast<std::string>(msg->track_id) + "/tracker";
			// 		std::string tracker_frame_id = "/perception/pipeline" + boost::lexical_cast<std::string>(7) + "/tracker";

						
			// 		std::string _arm_frame_id = "/perception/tabletop_segmentation/arm";
			// 		transf_arm.setOrigin( tf::Vector3(transf_.getOrigin().x()+ arm_reference_frame_offset_x_, 
			// 						    transf_.getOrigin().y()+ arm_reference_frame_offset_y_,
			// 						    transf_.getOrigin().z()+ arm_reference_frame_offset_z_) );
			// 
		
		
		
		/* _____________________
		|                     |
		|  Offline descriptor |
		|_____________________| */
		
// 		transf_object.setOrigin( tf::Vector3(0, 0, 0) );
// 		tf::Quaternion rotation;
// 		//rotation.setRPY(0, 0, 1.57);
// 		rotation.setRPY(0, 0, 0);	
// 		transf_object.setRotation(rotation);
// 		
// 		_br->sendTransform(StampedTransform(transf_object, Time::now(), "/odom_combined", tracker_frame_id));

		
		if (1)
		{
			boost::shared_ptr<pcl::PointCloud<T> > pca_object_view (new PointCloud<PointT>);
			boost::shared_ptr<PointCloud<PointT> > pca_pc (new PointCloud<PointT>); 
			vector < boost::shared_ptr<pcl::PointCloud<PointT> > > vector_of_projected_views;
			double largest_side = 0;
			int  sign = 1;
			vector <float> view_point_entropy;
			string std_name_of_sorted_projected_plane;
			vector< float > object_description;
			vector< float > object_description1Dvariance;

			Eigen::Vector3f center_of_bbox ;
			center_of_bbox(0,0) = msg->dimensions.x; center_of_bbox(0,1) = msg->dimensions.y;center_of_bbox(0,2) = msg->dimensions.z;
			
			
			
			///offline //TODO : put a parameter for offline and offline
			// compuet_object_description( target_pc,
			// 			    adaptive_support_lenght,
			// 			    global_image_width,
			// 			    threshold,
			// 			    number_of_bins,
			// 			    pca_object_view,
			// 			    center_of_bbox,
			// 			    vector_of_projected_views, 
			// 			    largest_side, 
			// 			    sign,
			// 			    view_point_entropy,
			// 			    std_name_of_sorted_projected_plane,
			// 			    object_description
			// 			  );
			
			///real_demo //TODO : put a parameter for offline and offline
			compuet_object_description_real_demo(   target_pc,
													adaptive_support_lenght,
													global_image_width,
													threshold,
													number_of_bins,
													msg->dimensions,
													vector_of_projected_views, 
													largest_side, 
													sign,
													view_point_entropy,
													std_name_of_sorted_projected_plane,
													object_description )	;
// 

// 			set_neat_visualization_marker_array_object_descriptor_vector_offline( target_pc,
// 				/*output_pc*/pca_object_view,
// 				center_of_bbox,
// 				vector_of_projected_views,
// 				msg->header.frame_id,
// 				msg -> track_id, 
// 				largest_side, 
// 				sign,
// 				view_point_entropy,
// 				std_name_of_sorted_projected_plane
// 				);	
// 					
// 			set_neat_visualization_marker_array_object_descriptor_vector( target_pc,
// 										      /*output_pc*/pca_object_view,
// 										      vector_of_projected_views,
// 										      msg->header.frame_id,
// 										      msg -> track_id, 
// 										      largest_side, 
// 										      sign,
// 										      view_point_entropy,
// 										      std_name_of_sorted_projected_plane );
			
			// // 			  
// 		      cout<< "\n GOOD description for the given object = [ ";
// 		      SITOV object_representation;
// 		      for (size_t i = 0; i < object_description.size(); i++)
// 		      {
// 			  object_representation.spin_image.push_back(object_description.at(i));
// 			  cout<< object_description.at(i)<<",";
// 		      }
// 		      cout<< "];\n";
			
		 
		      
// 		    #define SORT_BY_ENTROPY_AND_1D_VARIANCE			0
// 		    #define SORT_BY_ENTROPY_AND_2D_STDDEV_SUM			1
// 		    #define SORT_BY_DISTINCTIVENESS				2
// 		    #define SORT_BY_ENTROPY_AND_2D_VARIANCE_SUM		3
// 		    #define SORT_BY_ENTROPY_AND_AVERAGE_DISTANCE		4 
/*		   compuet_object_description_2D_variance( target_pc,
					      adaptive_support_lenght,
					      global_image_width,
					      threshold,
					      number_of_bins,
					      pca_object_view,
					      center_of_bbox,
					      vector_of_projected_views, 
					      largest_side, 
					      sign,
					      view_point_entropy,
					      std_name_of_sorted_projected_plane,
					      object_description, 
					      SORT_BY_ENTROPY_AND_2D_VARIANCE_SUM); */  
		      
		    // cout<< "\n GOOD description based on 2D variance for the given object = [ ";
			SITOV object_representation;
			for (size_t i = 0; i < object_description.size(); i++)
			{
				object_representation.spin_image.push_back(object_description.at(i));
				// cout<< object_description.at(i)<<",";
			}
		    // cout<< "];\n";


			SITOV deep_representation_sitov;
			/// call deep learning service to represent the given GOOD description as vgg16  
			race_deep_learning_feature_extraction::deep_representation srv;
			srv.request.good_representation = object_representation.spin_image;
			if (deep_learning_server.call(srv))
			{
				pp.info(std::ostringstream().flush() << "################ receive server responce with size of " << srv.response.deep_representation.size() );
				for (size_t i = 0; i < srv.response.deep_representation.size(); i++)
				{
					deep_representation_sitov.spin_image.push_back(srv.response.deep_representation.at(i));
				}
			}
			else
			{
				ROS_ERROR("Failed to call deep learning service");
			}
						
		                
        beginProc = ros::Time::now();

        //Declare SITOV (Spin Images of Tracked Object View)
        SITOV _sitov;

        //Declare RTOV (Representation of Tracked Object View)
        RTOV _rtov;
        _rtov.track_id = msg->track_id;
        _rtov.view_id = msg->view_id;

		ROS_INFO( "Track_id = %d,\tView_id = %d",  msg->track_id, msg->view_id );
		
		
        //declare the RTOV complete variable
        race_perception_msgs::CompleteRTOV _crtov;
        _crtov.track_id = msg->track_id;
        _crtov.view_id = msg->view_id;
        _crtov.ground_truth_name = msg->ground_truth_name.c_str();
		_crtov.pose_stamped = msg -> pose_stamped;
		_crtov.dimensions = msg -> dimensions;
		pp.info(std::ostringstream().flush() << "object_pose.x = %f " << msg -> pose_stamped.pose.position.x);
		pp.info(std::ostringstream().flush() << "object_pose.y = %f " << msg -> pose_stamped.pose.position.y);
		pp.info(std::ostringstream().flush() << "object_pose.z = %f " << msg -> pose_stamped.pose.position.z);

		ROS_INFO("ground_truth_name = %s", msg->ground_truth_name.c_str());

        //Add the object view representation in msg_out to put in the DB
		//_sitov = object_representation; //copy spin images
		_sitov = deep_representation_sitov; //copy deep representation into the msg
		_sitov.track_id = msg->track_id; //copy track_id
		_sitov.view_id = msg->view_id; //copy view_id
		_sitov.spin_img_id = 1; //copy spin image id

		//Addd sitov to completertov sitov list
		_crtov.sitov.push_back(_sitov);
        _crtov.is_key_view = msg->is_key_view;
        
        /* _____________________________________________
        |                                            |
        |  Write features to DB based on TID and VID |
        |____________________________________________| */
                
		// if (msg->is_key_view && !deep_learning) //add sitovs to the DB only if this is a key view
		if (msg->is_key_view) //add sitovs to the DB only if this is a key view
		{
		    //Serialize to add to DB
		    uint32_t serial_size = ros::serialization::serializationLength(_sitov);
		    boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
		    PerceptionDBSerializer<boost::shared_array<uint8_t>, SITOV>::serialize(buffer, _sitov, serial_size);	
		    leveldb::Slice s((char*)buffer.get(), serial_size);
		    std::string key = _pdb->makeSIKey(key::SI, msg->track_id, msg->view_id, 1 );

		    //Put slice to the DB
		    _pdb->put(key, s); 

		    //Add to the list of SITOV keys for this RTOV
		    _rtov.sitov_keys.push_back(key);
		    buffer.reset();
		}

        //Add RTOV to the DB (only if this is a key view)
        //if (msg->is_key_view && !deep_learning)            
		if (msg->is_key_view)                 
        {
            uint32_t serial_size = ros::serialization::serializationLength(_rtov);
            boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
            PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::serialize(buffer, _rtov, serial_size);	
            leveldb::Slice s((char*)buffer.get(), serial_size);
            std::string key = _pdb->makeKey(key::RV, msg->track_id, msg->view_id);

            //Put slice to the db
            _pdb->put(key, s);
            buffer.reset();
        }


        //Publish the CompleteRTOV to recognition
        _p_crtov_publisher->publish (_crtov);

        //Toc
        ros::Duration duration = (ros::Time::now() - beginProc);
        double duration_sec = duration.toSec();
        pp.info(std::ostringstream().flush() << "Write the features to DB took " << duration_sec << " secs");
		
					
		//add guassian noise 
// 		double standard_deviation=0.003;
// 		boost::shared_ptr<PointCloud<PointT> > output_pc (new PointCloud<PointT>);
// 		
// 		AddingGaussianNoise (	target_pc, 
// 					standard_deviation,
// 					output_pc);
// 			      
		
		
		//downSampling
// 		float downsampling_voxel_size= 0.01;
// 		boost::shared_ptr<PointCloud<PointT> > output_pc (new PointCloud<PointT>);
// 		downSampling ( target_pc, 		
// 				downsampling_voxel_size,
// 				output_pc);	
    
// 		set_neat_visualization_marker_array_object_descriptor_vector_offline( target_pc,
// 											/*output_pc*/pca_object_view,
// 											center_of_bbox,
// 											vector_of_projected_views,
// 											msg->header.frame_id,
// 											msg -> track_id, 
// 											largest_side, 
// 											sign,
// 											view_point_entropy,
// 											std_name_of_sorted_projected_plane
// 											);	
		
		ROS_INFO("before visualization dimension of largest_side =  %f ", largest_side);

		set_neat_visualization_marker_array_object_descriptor_vector(   target_pc,
																		vector_of_projected_views,
																		msg->header.frame_id,
																		msg -> track_id, 
																		largest_side, 
																		sign,
																		view_point_entropy,
																		std_name_of_sorted_projected_plane );	
		
        pp.printCallback();

// 		
// 		set_neat_visualization_marker_array_object_descriptor( vector_of_projected_views,
// 									   msg->header.frame_id,
// 									    msg -> track_id);

		}
		
		
		/* _____________________________________________________
		|    				    	                 		    |
		|  compute bounding box dimensions fo the given object  |
		|_______________________________________________________| */

// 		beginProc = ros::Time::now();
// 		//initial_cloud_ref = cloud_reference;
// 		geometry_msgs::Vector3 dimensions;
// 		compute_bounding_box_dimensions(target_pc, dimensions);
// 		ROS_INFO("box dimensions (x, y, z) = (%f, %f, %f) ", dimensions.x, dimensions.y, dimensions.z);
// 		
//                 //get toc
//                 duration = ros::Time::now() - beginProc;
//                 duration_sec = duration.toSec();
//                 ROS_INFO("Compute bounding box for given object took %f secs", duration_sec );

            }


            /* _______________________________
			|                                 |
			|           ACCESSORS             |
			|_________________________________| */
	    
	    template <typename T>
	    int set_neat_visualization_marker_array_object_descriptor_vector_offline( boost::shared_ptr<pcl::PointCloud<T> > object_view,
											boost::shared_ptr<pcl::PointCloud<T> > pca_object_view,
											Eigen::Vector3f center_of_bbox,
											vector < boost::shared_ptr<pcl::PointCloud<T> > >  all_projected_views, 
											string object_frame_id, 
											unsigned int TID , 
											double largest_side, 
											int sign,
											vector <float> view_point_entropy,
											string std_name_of_sorted_projected_plane )
	    {


    
	      
		std::string tracker_frame_id = "/perception/pipeline" + boost::lexical_cast<std::string>(TID) + "/tracker";
		ROS_INFO("tracker_frame_id = %s", tracker_frame_id.c_str());
		visualization_msgs::MarkerArray marker_array; 
		//geometry_msgs::Point p;
		double duration =1000;
		bool locked = true;
		bool finish= true;
		//float center_of_projected_cloud_x=0 , center_of_projected_cloud_y=0 , center_of_projected_cloud_z=0 ;
		//geometry_msgs::Point center_of_projected_cloud;
		//geometry_msgs::Vector3 dimensions;
		//geometry_msgs::Point p;
		// int view_i = 1;
		
		
		
		//////////////////////////// working with a dataset /////////////////////////////
		
		geometry_msgs::Point center_of_boundingbox_cloud;
// 		center_of_boundingbox_cloud.x=center_of_bbox(0,0);
// 		center_of_boundingbox_cloud.y=center_of_bbox(0,1);
// 		center_of_boundingbox_cloud.z=center_of_bbox(0,2);

			
		geometry_msgs::Point center_of_gravity_cloud;
		geometry_msgs::Vector3 dimensions;
		geometry_msgs::Point p;
		float center_of_gravity_cloud_x=0 , center_of_gravity_cloud_y=0 , center_of_gravity_cloud_z=0 ;
							
		compute_bounding_box_dimensions(pca_object_view, dimensions);
		center_of_boundingbox_cloud.x=dimensions.x/2;
		center_of_boundingbox_cloud.y=dimensions.y/2;
		center_of_boundingbox_cloud.z=dimensions.z/2;
		/* ____________________________________
		|                                     |
		|   Draw  PCA Point Cloud of object   |
		|_____________________________________| */
	      
		if (1)
		{
		      visualization_msgs::Marker marker;
		      //marker.header.frame_id = object_frame_id;
		      marker.header.stamp = ros::Time();		
		      marker.frame_locked = locked;
		      marker.header.frame_id = tracker_frame_id;

		      marker.ns = "pca object view";
		      marker.id = TID*20;
		      marker.type = visualization_msgs::Marker::POINTS;
		      marker.lifetime = Duration(duration);
		      marker.action = visualization_msgs::Marker::ADD;
		      marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
		      marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
		      marker.color.r = 0.5;  marker.color.g = 0.5; marker.color.b = 0.5; 	marker.color.a = 0.8;
		      marker.color.a = 1;
		      marker.scale.x = 0.002; marker.scale.y = 0.002; marker.scale.z = 0.002; 
		      
		      for (size_t i=0; i< pca_object_view->points.size(); i++)
		      {
			  //added recently
// 			  p.x = sign * pca_object_view->points.at(i).x;
// 			  p.y = sign * pca_object_view->points.at(i).y;
// 			  
			  p.x = pca_object_view->points.at(i).x;
			  p.y = pca_object_view->points.at(i).y;
			  p.z = pca_object_view->points.at(i).z;

			  center_of_gravity_cloud_x += p.x;
			  center_of_gravity_cloud_y += p.y;
			  center_of_gravity_cloud_z += p.z;

			  marker.points.push_back(p);
		      }

		      center_of_gravity_cloud.x= center_of_gravity_cloud_x/object_view->points.size();
		      center_of_gravity_cloud.y= center_of_gravity_cloud_y/object_view->points.size();
		      center_of_gravity_cloud.z= center_of_gravity_cloud_z/object_view->points.size();
		      marker_array.markers.push_back(marker);
		  
		}    

		/* _________________________________
		|                                 |
		|   Draw  Point Cloud of object   |
		|_________________________________| */
	      
		if (1)
		{
	  
	    
		  visualization_msgs::Marker marker;
		  //marker.header.frame_id = object_frame_id;
		  marker.header.stamp = ros::Time();		
		  marker.frame_locked = locked;
		  marker.header.frame_id = tracker_frame_id;

		  marker.ns = "object view";
		  marker.id = TID*1000;
		  marker.type = visualization_msgs::Marker::POINTS;
		  marker.lifetime = Duration(duration);
		  marker.action = visualization_msgs::Marker::ADD;
		  marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;

		  marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;

		  marker.color.r = 0.5;  marker.color.g = 0.5; marker.color.b = 0.0; 	marker.color.a = 1;
		  marker.scale.x = 0.002; marker.scale.y = 0.002; marker.scale.z = 0.002; marker.color.a = 1;
		  //marker.color.r = 1.0; marker.color.g = 1.0;	marker.color.b = 1.0; marker.color.a = 1.0; 
		  
		  for (size_t i=0; i< object_view->points.size(); i++)
		  {
		      p.x = object_view->points.at(i).x;
		      p.y = object_view->points.at(i).y;
		      p.z = object_view->points.at(i).z;
		      
		      marker.points.push_back(p);
		  }

		  marker_array.markers.push_back(marker);
		  
		  
			    
		}    
			    
			    
		
		 	/* _______________________________
			|                                 |
			|          DRAW BB WIREFRAME      |
			|_________________________________| */
	      	if (1)
	      	{
	      		visualization_msgs::Marker marker;
	      		marker.header.frame_id = tracker_frame_id;
	      		marker.header.stamp = ros::Time();
	      
	      		marker.ns = "wireframe_boundingbox";
	      		marker.id = TID*100;
	      		marker.frame_locked = locked;
	      		marker.type = visualization_msgs::Marker::LINE_LIST;
	      		//if (finish)
	      		//marker.action = visualization_msgs::Marker::DELETE;
	      		//else
	      		marker.action = visualization_msgs::Marker::ADD;
	      		marker.lifetime = Duration(duration);
	      
				marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;

// 		      
			
// 			marker.pose.position = center_of_boundingbox_cloud;
			marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
			    				
			
	      		marker.scale.x = 0.005; 
	      		double x = dimensions.x/2; 
	      		double y = dimensions.y/2; 
	      		double z = dimensions.z/2; 
	      
	      		marker.color = cm->color(TID);
	      		marker.color.r = 0.5;
	      		marker.color.g = 0.5;
	      		marker.color.b = 0.5;
			marker.color.a = 0.1;
	      		//marker
	      		if (finish)
	      		{
	      			marker.color.r = 0.1;
	      			marker.color.g = 0.1;
	      			marker.color.b = 0.1;
	      		}

	      		geometry_msgs::Point p;
	      		p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
	      		p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);
	      		p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
	      		p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
	      		p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
	      		p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
	      		p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
	      		p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);
	      
	      		p.x =  x; p.y =  -y; p.z =  z; marker.points.push_back(p);
	      		p.x = -x; p.y =  -y; p.z =  z; marker.points.push_back(p);
	      		p.x =  x; p.y =  -y; p.z = -z; marker.points.push_back(p);
	      		p.x = -x; p.y =  -y; p.z = -z; marker.points.push_back(p);
	      		p.x =  x; p.y =  -y; p.z = -z; marker.points.push_back(p);
	      		p.x =  x; p.y =  -y; p.z =  z; marker.points.push_back(p);
	      		p.x = -x; p.y =  -y; p.z = -z; marker.points.push_back(p);
	      		p.x = -x; p.y =  -y; p.z =  z; marker.points.push_back(p);
	      
	      		p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
	      		p.x =  x; p.y = -y; p.z =  z; marker.points.push_back(p);
	      		p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
	      		p.x =  x; p.y = -y; p.z = -z; marker.points.push_back(p);
	      
	      		p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);
	      		p.x = -x; p.y = -y; p.z =  z; marker.points.push_back(p);
	      		p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
	      		p.x = -x; p.y = -y; p.z = -z; marker.points.push_back(p);
	      
	      		marker_array.markers.push_back(marker);
	      	}
	      
	      
	      	/* _________________________________
	      	   |                                 |
	      	   |             DRAW BBOX           |
	      	   |_________________________________| */
	      	if (1)
	      	{
	      		visualization_msgs::Marker marker;
	      		marker.header.frame_id = tracker_frame_id;
	      		marker.header.stamp = ros::Time();
	      
	      		marker.ns = "boundingbox";
	      		marker.id = TID*290;
	      		marker.type = visualization_msgs::Marker::CUBE;
	      		marker.frame_locked = locked;
	      		//if (finish)
	      		//marker.action = visualization_msgs::Marker::DELETE;
	      		//else
	      		marker.action = visualization_msgs::Marker::ADD;
	      		marker.lifetime = Duration(duration);
	      
// 			marker.pose.position = center_of_boundingbox_cloud;
// 			marker.pose.position = center_of_gravity_cloud;

			marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;

			marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
	      		
			marker.scale.x = dimensions.x; 
	      		marker.scale.y = dimensions.y; 
	      		marker.scale.z = dimensions.z; 
	      

			marker.color = cm->color(TID);
			marker.color.a = 0.2;
			
			if (finish)
	      		{
	      			marker.color.r = 0.9;
	      			marker.color.g = 0.0;
	      			marker.color.b = 0.0;
	      		}
	      
	      		marker_array.markers.push_back(marker);
	      	}
		
		
		
		
		for (int view_i = 0; view_i< all_projected_views.size(); view_i++)			
		{
			geometry_msgs::Point center_of_projected_cloud;
			geometry_msgs::Vector3 dimensions;
			geometry_msgs::Point p;
			float center_of_projected_cloud_x=0 , center_of_projected_cloud_y=0 , center_of_projected_cloud_z=0 ;
			/* _________________________________
			  |                                 |
			  |   Draw Projected Point Cloud    |
			  |_________________________________| */
			if (1)
			{
		    
		      
			    visualization_msgs::Marker marker;
			    //marker.header.frame_id = object_frame_id;
			    marker.header.stamp = ros::Time();		
			    marker.frame_locked = locked;
			    marker.header.frame_id = tracker_frame_id;

			    marker.ns = "projected views";
			    marker.id = TID*520+view_i;
			    marker.type = visualization_msgs::Marker::POINTS;
			    marker.lifetime = Duration(duration);
			    marker.action = visualization_msgs::Marker::ADD;
			    marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
			    marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
			    marker.color = cm->color(TID);
			    marker.scale.x = 0.005; marker.scale.y = 0.005; marker.scale.z = 0.005; marker.color.a = 1;
			    //marker.color.r = 1.0; marker.color.g = 1.0;	marker.color.b = 1.0; marker.color.a = 1.0; 
			    
			    //marker.points.erase(marker.points.begin(), marker.points.end());
			    geometry_msgs::Point p;

			    for (size_t i=0; i<all_projected_views.at(view_i)-> points.size(); i++)
			    {
// 				p.x = sign * all_projected_views.at(view_i)->points.at(i).x;
// 				p.y = sign * all_projected_views.at(view_i)->points.at(i).y;
// 				p.z = all_projected_views.at(view_i)->points.at(i).z;
// 				marker.points.push_back(p);

				//all_projected_views.at(view_i)->points.at(i).x = sign * all_projected_views.at(view_i)->points.at(i).x;
				//all_projected_views.at(view_i)->points.at(i).y = sign * all_projected_views.at(view_i)->points.at(i).y;
				
				
// 				if (view_i == 0)
// 				{
// 				    all_projected_views.at(view_i)->points.at(i).x = all_projected_views.at(view_i)->points.at(i).x;
// 				    all_projected_views.at(view_i)->points.at(i).y = sign * all_projected_views.at(view_i)->points.at(i).y;
// 				    //all_projected_views.at(view_i)->points.at(i).x =  all_projected_views.at(view_i)->points.at(i).x;
// 				    //all_projected_views.at(view_i)->points.at(i).y =  all_projected_views.at(view_i)->points.at(i).y;
// 				}
// 				else if (view_i == 1)
// 				{
// 				    all_projected_views.at(view_i)->points.at(i).x = sign * all_projected_views.at(view_i)->points.at(i).x;
// 				    all_projected_views.at(view_i)->points.at(i).y = all_projected_views.at(view_i)->points.at(i).y;
// 				  
// 				}
// 				else
// 				{
// 				    all_projected_views.at(view_i)->points.at(i).x = sign * all_projected_views.at(view_i)->points.at(i).x;
// 				    all_projected_views.at(view_i)->points.at(i).y = sign * all_projected_views.at(view_i)->points.at(i).y;
// 				  
// 				}
    
				all_projected_views.at(view_i)->points.at(i).z = sign * all_projected_views.at(view_i)->points.at(i).z;
				p.x = all_projected_views.at(view_i)->points.at(i).x;
				p.y = all_projected_views.at(view_i)->points.at(i).y;
				//p.z = sign * all_projected_views.at(view_i)->points.at(i).z;
								
				p.z = all_projected_views.at(view_i)->points.at(i).z;

								
				marker.points.push_back(p);
				center_of_projected_cloud_x += p.x;
				center_of_projected_cloud_y += p.y;
				center_of_projected_cloud_z += p.z;
			    }

			    marker_array.markers.push_back(marker);
			    
			    center_of_projected_cloud.x= center_of_projected_cloud_x/all_projected_views.at(view_i)->points.size();
			    center_of_projected_cloud.y= center_of_projected_cloud_y/all_projected_views.at(view_i)->points.size();
			    center_of_projected_cloud.z= center_of_projected_cloud_z/all_projected_views.at(view_i)->points.size();
			    
			}
			
			  /* _________________________________
			    |                                 |
			    |           Draw Plane            |
			    |_________________________________| */
			  if (1)
			  {
				  visualization_msgs::Marker marker;
				  marker.header.frame_id = tracker_frame_id;
				  marker.header.stamp = ros::Time();
			
				  marker.ns = "Plane";
				  marker.id = TID*150+view_i;
				  marker.type = visualization_msgs::Marker::CUBE;
				  marker.frame_locked = locked;
				  //if (finish)
				  //marker.action = visualization_msgs::Marker::DELETE;
				  //else
				  marker.action = visualization_msgs::Marker::ADD;
				  marker.lifetime = Duration(duration);
			
				  marker.pose.position =center_of_projected_cloud;

				  //marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;

				  //compute_bounding_box_dimensions(all_projected_views.at(view_i), dimensions);
				  //ROS_INFO("Visuzlize box dimensions (x, y, z) = (%f, %f, %f) ", dimensions.x, dimensions.y, dimensions.z);
				  
				  if (view_i == 0)
				  {
				  	marker.scale.x = 0.0001;
					marker.scale.y = largest_side; 
					marker.scale.z = largest_side;
					
					dimensions.x = 0.0001; 
					dimensions.y = largest_side; 
					dimensions.z = largest_side;
				  }
				  else if (view_i ==1)
				  {
					marker.scale.x = largest_side; 
					marker.scale.y =  0.0001;
					marker.scale.z = largest_side; 
					
					dimensions.x = largest_side ; 
					dimensions.y = 0.0001;
					dimensions.z = largest_side; 
			
				  } 
				  else
				  {
				      marker.scale.x = largest_side; 
				      marker.scale.y = largest_side; 
				      marker.scale.z = 0.0001;
				      
				      dimensions.x = largest_side; 
				      dimensions.y = largest_side; 
				      dimensions.z = 0.0001;
				  }
				  			  
				  marker.color = cm->color(TID);		  
				  marker.color.a = 0.3;
						    
		
				  marker_array.markers.push_back(marker);
			  }
			  

				  
				  /* _________________________________
				    |                                 |
				    |         DRAW WIREFRAME          |
				    |_________________________________| */
				  if (1)
				  {
				    
					  visualization_msgs::Marker marker;
					  marker.header.frame_id = tracker_frame_id;
					  marker.header.stamp = ros::Time();
				
					  marker.ns = "wireframe";
					  marker.id = TID*170+view_i;
					  marker.frame_locked = locked;
					  marker.type = visualization_msgs::Marker::LINE_LIST;
					  //if (finish)
					  //marker.action = visualization_msgs::Marker::DELETE;
					  //else
					  marker.action = visualization_msgs::Marker::ADD;
					  marker.lifetime = Duration(duration);
							  
					 marker.pose.position =center_of_projected_cloud;
					 // marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;

					  //marker.pose = _tracked_object_msg.bounding_box.pose_stamped.pose;
				
					  marker.scale.x = 0.003; 

					  marker.color.a = 0.5;
					  marker.color.r = 0.5;
					  marker.color.g = 0.5;
					  marker.color.b = 0.5;
					  //marker
					  if (finish)
					  {
						  marker.color.r = 0.1;
						  marker.color.g = 0.1;
						  marker.color.b = 0.1;
					  }

					  
					  
					double x = dimensions.x/2; 
					double y = dimensions.y/2; 
					double z = dimensions.z/2; 
// 					double x = largest_side/2; 
// 					double y = largest_side/2; 
// 					double z = largest_side/2; 
					
					//for Ortoghraphic net, we just want to show the palne, so k =1;
					int k = 1; //number_of_bins;
					
					double interval_x = dimensions.x/k; 
					double interval_y = dimensions.y/k; 
					double interval_z = dimensions.z/k; 

// 					double interval_x = largest_side/k; 
// 					double interval_y = largest_side/k; 
// 					double interval_z = largest_side/k; 
				    
					if (view_i == 0)//for x
					{
					  //ROS_INFO("YOZ WIREFRAME plane size (x, y, z) = (%f, %f, %f) ", x, y, z);   
					  // Page YOZ for Y axis
					      for (int i=0; i<= k ; i++)  
					      {	
						    p.x = x; p.y = -y + i*interval_y; p.z = -z; 
						    marker.points.push_back(p);
						    p.x = x; p.y = -y + i*interval_y ; p.z =  z; 
						    marker.points.push_back(p);	
					      }
					      
					      // Page YOZ for Z axis
					      for (int i=0; i<= k ; i++)  
					      {
						    p.x = x; p.y = y; p.z = -z + i*interval_z; 
						    marker.points.push_back(p);
						    p.x = x; p.y = -y ; p.z = -z + i*interval_z; ; 
						    marker.points.push_back(p);
					      }
					      marker_array.markers.push_back(marker);
					      
					}
					
					else if (view_i == 1)//for y
					{
					  //ROS_INFO("XOZ WIREFRAME plane size (x, y, z) = (%f, %f, %f) ", x, y, z);   

					      // Page XOZ for X axis
					      for (int i=0; i<= k ; i++)  
					      {	
						    p.x = -x+ i*interval_x; p.y = y; p.z = -z; 
						    marker.points.push_back(p);
						    p.x = -x+ i*interval_x; p.y = y ; p.z =  z; 
						    marker.points.push_back(p);	
					      }
					      
					      // Page XOZ for Z axis
					      for (int i=0; i<= k ; i++)  
					      {
						    p.x = x; p.y = y; p.z = -z + i*interval_z; 
						    marker.points.push_back(p);
						    p.x = -x; p.y = y ; p.z = -z + i*interval_z; ; 
						    marker.points.push_back(p);
					      }
					      marker_array.markers.push_back(marker);

					}
					
					else //for z
					{
					  //ROS_INFO("XOY WIREFRAME plane size (x, y, z) = (%f, %f, %f) ", x, y, z);   
					      // Page XOY for X axis
					      for (int i=0; i<= k ; i++)  
					      {	
						    p.x = -x+ i*interval_x; p.y = -y; p.z = z; 
						    marker.points.push_back(p);
						    p.x = -x+ i*interval_x; p.y = y ; p.z = z; 
						    marker.points.push_back(p);	
					      }
					      
					      // Page XOY for Y axis
					      for (int i=0; i<= k ; i++)  
					      {	
						    p.x = -x; p.y = -y + i*interval_y; p.z = z; 
						    marker.points.push_back(p);
						    p.x = x; p.y = -y + i*interval_y ; p.z = z; 
						    marker.points.push_back(p);	
					      }
					      marker_array.markers.push_back(marker);
					      
					}
	      		      } 
	      		     

				
	      		     
				    /* _______________________
				    |                         |
				    |         histogram       |
				    |_________________________| */
				  if (1)
				  {
				    
				        visualization_msgs::Marker marker;
					//marker.header.frame_id = object_frame_id;
					marker.header.stamp = ros::Time();		
					marker.frame_locked = locked;
					marker.header.frame_id = tracker_frame_id;

					marker.ns = "histogram of projected views";
					marker.id = TID*10+view_i;
					marker.type = visualization_msgs::Marker::POINTS;
					marker.lifetime = Duration(duration);
					marker.action = visualization_msgs::Marker::ADD;
// 					marker.pose.position.x = 0; marker.pose.position.y = 0;	marker.pose.position.z = 0;
// 					marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
					marker.color = cm->color(TID);
					marker.scale.x = 0.01; marker.scale.y = 0.01; marker.scale.z = 0.01; marker.color.a = 1;
					marker.color.r = 0.0; marker.color.g = 0.0;	marker.color.b = 1.0; marker.color.a = 1.0; 
										  
					  
					double x = dimensions.x/2; 
					double y = dimensions.y/2; 
					double z = dimensions.z/2; 
					    
					int k =number_of_bins;
					
					double interval_x = dimensions.x/k; 
					double interval_y = dimensions.y/k; 
					double interval_z = dimensions.z/k; 

// 					double interval_x = largest_side/k; 
// 					double interval_y = largest_side/k; 
// 					double interval_z = largest_side/k; 
				    
					if (view_i == 0)//for x
					{
					  //ROS_INFO("YOZ histogram");   
					  for (int i=0; i<all_projected_views.at(view_i)-> points.size(); i++)
					  {
						geometry_msgs::Point p;
						//p.x = all_projected_views.at(view_i)->points.at(i).x + x;
						// 1/2 dimention is added to the real data to put all the data in positive scale between [0,2y][0-2z]
						p.y = all_projected_views.at(view_i)->points.at(i).y + sign * y;
						//p.y = all_projected_views.at(view_i)->points.at(i).y + y;
						p.z = all_projected_views.at(view_i)->points.at(i).z + z;
						//ROS_INFO("YOZ: P(y).(%i)= %f --- dy= %f", all_projected_views.at(view_i)->points.at(i).y, y );
						//ROS_INFO("(trunc(p.y / interval_y)= %f and (trunc(p.z / interval_z) =%f", trunc(p.y / interval_y), trunc(p.z / interval_z));
						if ((trunc(p.y / interval_y) == 0.0) and ((trunc(p.z / interval_z) == 0.0)))
						{
						    geometry_msgs::Point p;
						    p.x=all_projected_views.at(view_i)->points.at(i).x;
						    p.y=all_projected_views.at(view_i)->points.at(i).y;
						    p.z=all_projected_views.at(view_i)->points.at(i).z;
						    marker.points.push_back(p);
			
// 						    p.x=all_projected_views.at(view_i)->points.at(i).x;
// 						    p.y= - all_projected_views.at(view_i)->points.at(i).y;
// 						    p.z=all_projected_views.at(view_i)->points.at(i).z;
// 						    marker.points.push_back(p);
// 						    
						}
					    }
					    marker_array.markers.push_back(marker);
					      
					}
					
					else if (view_i == 1)//for y
					{
					  //ROS_INFO("XOZ histogram");   
					  for (int i=0; i<all_projected_views.at(view_i)-> points.size(); i++)
					  {					    
						geometry_msgs::Point p;
						// 1/2 dimention is added to the real data to put all the data in positive scale between [0,2x][0-2z]
						p.x = all_projected_views.at(view_i)->points.at(i).x + sign * x;
						//p.x = all_projected_views.at(view_i)->points.at(i).x + x;
						//p.y = all_projected_views.at(view_i)->points.at(i).y;
						p.z = all_projected_views.at(view_i)->points.at(i).z + z;
						//ROS_INFO("XOZ: P(x).(%i)= %f --- dx= %f", all_projected_views.at(view_i)->points.at(i).x, x );
						//ROS_INFO("(trunc(p.x / interval_x)= %f and (trunc(p.z / interval_z) =%f", trunc(p.x / interval_x), trunc(p.z / interval_z));
						
						if ((trunc(p.x / interval_x) == 0.0) and ((trunc(p.z / interval_z) == 0.0)))
						{
						    geometry_msgs::Point p;
						    p.x=all_projected_views.at(view_i)->points.at(i).x;
						    p.y=all_projected_views.at(view_i)->points.at(i).y;
						    p.z=all_projected_views.at(view_i)->points.at(i).z;
						    marker.points.push_back(p);
						    
// 						    p.x= - all_projected_views.at(view_i)->points.at(i).x;
// 						    p.y=all_projected_views.at(view_i)->points.at(i).y;
// 						    p.z=all_projected_views.at(view_i)->points.at(i).z;
// 						    marker.points.push_back(p);
						}
					    }
					    marker_array.markers.push_back(marker);

					}
					
					else //for z
					{
					  //ROS_INFO("XOY histogram");   					       
					  for (int i=0; i<all_projected_views.at(view_i)-> points.size(); i++)
					  {
						geometry_msgs::Point p;
						// 1/2 dimention is added to the real data to put all the data in positive scale between [0,2x][0-2y]
						p.x = all_projected_views.at(view_i)->points.at(i).x + sign * x;						
						p.y = all_projected_views.at(view_i)->points.at(i).y + sign * y;
						//p.x = all_projected_views.at(view_i)->points.at(i).x +  x;						
						//p.y = all_projected_views.at(view_i)->points.at(i).y +  y;
						//p.z = all_projected_views.at(view_i)->points.at(i).z + z;
						//ROS_INFO("XOY: P(x).(%i)= %f --- dx= %f", all_projected_views.at(view_i)->points.at(i).x, x );
						//ROS_INFO("(trunc(p.x / interval_x)= %f and (trunc(p.y / interval_y) =%f", trunc(p.x / interval_x), trunc(p.y / interval_y));
						if ((trunc(p.x / interval_x) == 0.0) and ((trunc(p.y / interval_y) == 0.0)))
						{
						    geometry_msgs::Point p;
						    p.x=all_projected_views.at(view_i)->points.at(i).x;
						    p.y=all_projected_views.at(view_i)->points.at(i).y;
						    p.z=all_projected_views.at(view_i)->points.at(i).z;
						    marker.points.push_back(p);
						    
// 						    p.x= - all_projected_views.at(view_i)->points.at(i).x;
// 						    p.y= - all_projected_views.at(view_i)->points.at(i).y;
// 						    p.z=all_projected_views.at(view_i)->points.at(i).z;
// 						    marker.points.push_back(p);
						}
					    }
					    marker_array.markers.push_back(marker);
					      
					}
	      		      } 

	      		      
	      		      /* _________________________________
				|                                 |
				|         DRAW TEXT INFO          |
				|_________________________________| */
				if (1)
				{
					
					visualization_msgs::Marker marker;
					//marker.header.frame_id = object_frame_id;
					marker.header.stamp = ros::Time();		
					marker.frame_locked = locked;
					marker.header.frame_id = tracker_frame_id;

//					marker.pose.position =center_of_boundingbox_cloud;
					marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;

					marker.ns = "sign";
					marker.id = TID*10+view_i;
					marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					marker.lifetime = Duration(duration);
					marker.action = visualization_msgs::Marker::ADD;
					//marker.pose.position.x = 0 + marker.pose.position.x*0.9 + 0;
					//marker.pose.position.y = 0 + marker.pose.position.y*0.9 + 0.1;
					marker.pose.position.z = largest_side + 0.1;
					marker.scale.z = 0.02; 
					marker.color.r  = 0; marker.color.g  = 0; marker.color.b  = 0; marker.color.a = 1;
					marker.text = "Sign = " + boost::lexical_cast<std::string>(sign);
					marker_array.markers.push_back(marker);
				}

				if (1)
				{
					
					visualization_msgs::Marker marker;
					//marker.header.frame_id = object_frame_id;
					marker.header.stamp = ros::Time();		
					marker.frame_locked = locked;
					marker.header.frame_id = tracker_frame_id;

					//marker.pose.position =center_of_gravity_cloud;
					marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;


					marker.ns = "entropy";
					marker.id = TID*100+view_i;
					marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					marker.lifetime = Duration(duration);
					marker.action = visualization_msgs::Marker::ADD;
					//marker.pose.position.x = 0 + marker.pose.position.x*0.9 + 0;
					//marker.pose.position.y = 0 + marker.pose.position.y*0.9 + 0.1;
					marker.pose.position.z = largest_side + 0.25;
					marker.scale.z = 0.02; 
					marker.color.r  = 1; marker.color.g  = 0; marker.color.b  = 0; marker.color.a = 1;
					marker.text = "H(YoZ) = " + boost::lexical_cast<std::string>(view_point_entropy.at(0))+
						      "\nH(XoZ) = " + boost::lexical_cast<std::string>(view_point_entropy.at(1))+
						      "\nH(XoY) = " + boost::lexical_cast<std::string>(view_point_entropy.at(2));
					marker_array.markers.push_back(marker);
				}

				if (1)
				{
					
					visualization_msgs::Marker marker;
					//marker.header.frame_id = object_frame_id;
					marker.header.stamp = ros::Time();		
					marker.frame_locked = locked;
					marker.header.frame_id = tracker_frame_id;

					//marker.pose.position =center_of_gravity_cloud;
					marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;


					marker.ns = "sorted projections";
					marker.id = TID*200+view_i;
					marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					marker.lifetime = Duration(duration);
					marker.action = visualization_msgs::Marker::ADD;
					//marker.pose.position.x = 0 + marker.pose.position.x*0.9 + 0;
					//marker.pose.position.y = 0 + marker.pose.position.y*0.9 + 0.1;
					marker.pose.position.z = largest_side + 0.3;
					marker.scale.z = 0.02; 
					marker.color.r  = 0; marker.color.g  = 0; marker.color.b  = 0; marker.color.a = 1;
					marker.text = std_name_of_sorted_projected_plane;
					marker_array.markers.push_back(marker);
				}

				
				
			      /* _______________________________
			      |                                 |
			      |           DRAW XYZ AXES         |
			      |_________________________________| */
				if (1)
				{	
					visualization_msgs::Marker marker;
					double axis_dimension = sign * 0.2;
					marker.header.frame_id = tracker_frame_id;
					marker.header.stamp = ros::Time();

					marker.frame_locked = locked;
					marker.type = visualization_msgs::Marker::LINE_STRIP;
					marker.action = visualization_msgs::Marker::ADD;
					marker.lifetime = Duration(duration);

					//marker.pose.position =center_of_boundingbox_cloud;
					marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;

					marker.scale.x = 0.01; marker.scale.y = 0.5; marker.scale.z = 4;

					//X axis
					marker.ns = "axes_x";
					marker.id = TID*10+view_i;;
					marker.color.r = 1.0; marker.color.g = 0.0;	marker.color.b = 0.0; marker.color.a = 1.0; //red color
					marker.points.erase(marker.points.begin(), marker.points.end());
					p.x = 0; p.y = 0; p.z = 0; 
					marker.points.push_back(p);
					p.x = 1 * axis_dimension; p.y = 0; p.z = 0; 
					marker.points.push_back(p);
					marker_array.markers.push_back(marker);

					//Y axis
					marker.ns = "axes_y";
					marker.id = TID*10+view_i;
					marker.color.r = 0.0; marker.color.g = 1.0;	marker.color.b = 0.0; marker.color.a = 1.0; //green color
					marker.points.erase(marker.points.begin(), marker.points.end());
					p.x = 0; p.y = 0; p.z = 0; 
					marker.points.push_back(p);
					p.x = 0; p.y = 1 * axis_dimension; p.z = 0; 
					marker.points.push_back(p);
					marker_array.markers.push_back(marker);

					//Z axis
					marker.ns = "axes_z";
					marker.id = TID*10+view_i;
					marker.color.r = 0.0; marker.color.g = 0.0;	marker.color.b = 1.0; marker.color.a = 1.0; //blue color
					marker.points.erase(marker.points.begin(), marker.points.end());
					p.x = 0; p.y = 0; p.z = 0; 

					marker.points.push_back(p);

					p.x = 0; p.y =0 ; p.z = 1 * sign * axis_dimension;

					
// 					if (sign > 0 )
// 					{
// 					  p.x = 0; p.y = 0; p.z = 1 * axis_dimension;  
// 					}
// 					else 
// 					{
// 					  p.x = 0; p.y = 0; p.z = 1 * -axis_dimension;
// 					}
					 
					marker.points.push_back(p);
					marker_array.markers.push_back(marker);
				}
				
				}
				


		      neat_marker_publisher.publish(marker_array);
		      return 1;


		  }
		
template <typename T>
	    int set_neat_visualization_marker_array_object_descriptor_vector( boost::shared_ptr<pcl::PointCloud<T> > object_view,
										vector < boost::shared_ptr<pcl::PointCloud<T> > >  all_projected_views, 
									        string object_frame_id, 
										unsigned int TID , 
										double largest_side, 
										int sign,
										vector <float> view_point_entropy,
										string std_name_of_sorted_projected_plane
										
									    )
	    {
	      
		std::string tracker_frame_id = "/perception/pipeline" + boost::lexical_cast<std::string>(TID) + "/tracker";

		ROS_INFO("dimension of largest_side =  %f ", largest_side);

		visualization_msgs::MarkerArray marker_array; 
		//geometry_msgs::Point p;
		double duration =2;
		bool locked = true;
		bool finish= true;
		//float center_of_projected_cloud_x=0 , center_of_projected_cloud_y=0 , center_of_projected_cloud_z=0 ;
		//geometry_msgs::Point center_of_projected_cloud;
		//geometry_msgs::Vector3 dimensions;
		//geometry_msgs::Point p;
		// int view_i = 1;
		
		
		
		//////////////////////////// working with a dataset /////////////////////////////
		geometry_msgs::Point center_of_boundingbox_cloud;
		geometry_msgs::Vector3 dimensions;
		geometry_msgs::Point p;
		float center_of_gravity_cloud_x=0 , center_of_gravity_cloud_y=0 , center_of_gravity_cloud_z=0 ;
// 		/* _________________________________
// 		|                                 |
// 		|   Draw  Point Cloud of object   |
// 		|_________________________________| */
// 	      
// 		if (1)
// 		{
// 	  
// 	    
// 		  visualization_msgs::Marker marker;
// 		  //marker.header.frame_id = object_frame_id;
// 		  marker.header.stamp = ros::Time();		
// 		  marker.frame_locked = locked;
// 		  marker.header.frame_id = tracker_frame_id;
// 
// 		  marker.ns = "object view";
// 		  marker.id = TID*100000000;
// 		  marker.type = visualization_msgs::Marker::POINTS;
// 		  marker.lifetime = Duration(duration);
// 		  marker.action = visualization_msgs::Marker::ADD;
// 		  marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
// 		  marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
// // 		  marker.color = cm->color(TID);
// 		  marker.color.r = 0;
// 		  marker.color.g = 0;
// 		  marker.color.b = 0;
// 		  marker.color.a = 1;
// 		  
// 		  marker.scale.x = 0.005; marker.scale.y = 0.005; marker.scale.z = 0.005; marker.color.a = 1;
// 		  //marker.color.r = 1.0; marker.color.g = 1.0;	marker.color.b = 1.0; marker.color.a = 1.0; 
// 		  
// 		  for (size_t i=0; i< object_view->points.size(); i++)
// 		  {
// 		      p.x = object_view->points.at(i).x;
// 		      p.y = object_view->points.at(i).y;
// 		      p.z = object_view->points.at(i).z;
// 		      
// 		      marker.points.push_back(p);
// 		      center_of_gravity_cloud_x += p.x;
// 		      center_of_gravity_cloud_y += p.y;
// 		      center_of_gravity_cloud_z += p.z;
// 		  }
// 
// 		  marker_array.markers.push_back(marker);
// 		  
// // 		  center_of_gravity_cloud.x= center_of_gravity_cloud_x/object_view->points.size();
// // 		  center_of_gravity_cloud.y= center_of_gravity_cloud_y/object_view->points.size();
// // 		  center_of_gravity_cloud.z= center_of_gravity_cloud_z/object_view->points.size();
// 			    
// 		}    
// 			    
			    
		
// 		 /* _________________________________
// 	      	   |                                 |
// 	      	   |          DRAW BB WIREFRAME      |
// 	      	   |_________________________________| */
// 	      	if (1)
// 	      	{
// 	      		visualization_msgs::Marker marker;
// 	      		marker.header.frame_id = tracker_frame_id;
// 	      		marker.header.stamp = ros::Time();
// 	      
// 	      		marker.ns = "wireframe";
// 	      		marker.id = TID*10000;
// 	      		marker.frame_locked = locked;
// 	      		marker.type = visualization_msgs::Marker::LINE_LIST;
// 	      		//if (finish)
// 	      		//marker.action = visualization_msgs::Marker::DELETE;
// 	      		//else
// 	      		marker.action = visualization_msgs::Marker::ADD;
// 	      		marker.lifetime = Duration(duration);
// 	      
// 			compute_bounding_box_dimensions(object_view, dimensions);
// 			center_of_boundingbox_cloud.x=dimensions.x/2;
// 			center_of_boundingbox_cloud.y=dimensions.y/2;
// 			center_of_boundingbox_cloud.z=dimensions.z/2;
// 
// 			marker.pose.position = center_of_boundingbox_cloud;	/*marker.pose.position.y = 0;	marker.pose.position.z = 0;*/
// 			marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
// 			    				
// 			
// 	      		marker.scale.x = 0.005; 
// 	      		double x = dimensions.x/2; 
// 	      		double y = dimensions.y/2; 
// 	      		double z = dimensions.z/2; 
// 	      
// 	      		marker.color = cm->color(TID);
// 	      		marker.color.r = 0.5;
// 	      		marker.color.g = 0.5;
// 	      		marker.color.b = 0.5;
// 			marker.color.a = 0.1;
// 	      		//marker
// 	      		if (finish)
// 	      		{
// 	      			marker.color.r = 0.1;
// 	      			marker.color.g = 0.1;
// 	      			marker.color.b = 0.1;
// 	      		}
// 
// 	      		geometry_msgs::Point p;
// 	      		p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);
// 	      		p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
// 	      		p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
// 	      		p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);
// 	      
// 	      		p.x =  x; p.y =  -y; p.z =  z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  -y; p.z =  z; marker.points.push_back(p);
// 	      		p.x =  x; p.y =  -y; p.z = -z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  -y; p.z = -z; marker.points.push_back(p);
// 	      		p.x =  x; p.y =  -y; p.z = -z; marker.points.push_back(p);
// 	      		p.x =  x; p.y =  -y; p.z =  z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  -y; p.z = -z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  -y; p.z =  z; marker.points.push_back(p);
// 	      
// 	      		p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
// 	      		p.x =  x; p.y = -y; p.z =  z; marker.points.push_back(p);
// 	      		p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
// 	      		p.x =  x; p.y = -y; p.z = -z; marker.points.push_back(p);
// 	      
// 	      		p.x = -x; p.y =  y; p.z =  z; marker.points.push_back(p);
// 	      		p.x = -x; p.y = -y; p.z =  z; marker.points.push_back(p);
// 	      		p.x = -x; p.y =  y; p.z = -z; marker.points.push_back(p);
// 	      		p.x = -x; p.y = -y; p.z = -z; marker.points.push_back(p);
// 	      
// 	      		marker_array.markers.push_back(marker);
// 	      	}
// 	      
// 	      
// 	      	/* _________________________________
// 	      	   |                                 |
// 	      	   |             DRAW BBOX           |
// 	      	   |_________________________________| */
// 	      	if (1)
// 	      	{
// 	      		visualization_msgs::Marker marker;
// 	      		marker.header.frame_id = tracker_frame_id;
// 	      		marker.header.stamp = ros::Time();
// 	      
// 	      		marker.ns = "boundingbox";
// 	      		marker.id = TID*200;
// 	      		marker.type = visualization_msgs::Marker::CUBE;
// 	      		marker.frame_locked = locked;
// 	      		//if (finish)
// 	      		//marker.action = visualization_msgs::Marker::DELETE;
// 	      		//else
// 	      		marker.action = visualization_msgs::Marker::ADD;
// 	      		marker.lifetime = Duration(duration);
// 	      
// 			marker.pose.position = center_of_boundingbox_cloud;
// 			marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
// 	      		
// 			marker.scale.x = dimensions.x; 
// 	      		marker.scale.y = dimensions.y; 
// 	      		marker.scale.z = dimensions.z; 
// 	      
// 
// 			marker.color = cm->color(TID);
// 			marker.color.a = 0.1;
// 			
// 			if (finish)
// 	      		{
// 	      			marker.color.r = 0.9;
// 	      			marker.color.g = 0.0;
// 	      			marker.color.b = 0.0;
// 	      		}
// 	      
// 	      		marker_array.markers.push_back(marker);
// 	      	}
		
		
		
		/* _________________________________
		|                                 |
		|   Draw Projected Point Cloud    |
		|_________________________________| */
			      
		for (int view_i = 0; view_i< all_projected_views.size(); view_i++)			
		{
			geometry_msgs::Point center_of_projected_cloud;
			geometry_msgs::Vector3 dimensions;
			geometry_msgs::Point p;
			float center_of_projected_cloud_x=0 , center_of_projected_cloud_y=0 , center_of_projected_cloud_z=0 ;

			if (1)
			{
		    
		      
			    visualization_msgs::Marker marker;
			    //marker.header.frame_id = object_frame_id;
			    marker.header.stamp = ros::Time();		
			    marker.frame_locked = locked;
			    marker.header.frame_id = tracker_frame_id;

			    marker.ns = "projected views";
			    marker.id = TID*32+view_i;
			    marker.type = visualization_msgs::Marker::POINTS;
			    marker.lifetime = Duration(duration);
			    marker.action = visualization_msgs::Marker::ADD;
// 			    marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
// 			    marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
			    marker.color = cm->color(TID);
			    marker.scale.x = 0.0035; marker.scale.y = 0.0035; marker.scale.z = 0.0035; marker.color.a = 1;
			    //marker.color.r = 1.0; marker.color.g = 1.0;	marker.color.b = 1.0; marker.color.a = 1.0; 
			    
			    //marker.points.erase(marker.points.begin(), marker.points.end());
			    geometry_msgs::Point p;

			    for (size_t i=0; i<all_projected_views.at(view_i)-> points.size(); i++)
			    {
// 				p.x = sign * all_projected_views.at(view_i)->points.at(i).x;
// 				p.y = sign * all_projected_views.at(view_i)->points.at(i).y;
// 				p.z = all_projected_views.at(view_i)->points.at(i).z;
// 				marker.points.push_back(p);

				all_projected_views.at(view_i)->points.at(i).x = sign * all_projected_views.at(view_i)->points.at(i).x;
				all_projected_views.at(view_i)->points.at(i).y = sign * all_projected_views.at(view_i)->points.at(i).y;
				p.x = all_projected_views.at(view_i)->points.at(i).x;
				p.y = all_projected_views.at(view_i)->points.at(i).y;
				p.z = all_projected_views.at(view_i)->points.at(i).z;
				
				marker.points.push_back(p);
				center_of_projected_cloud_x += p.x;
				center_of_projected_cloud_y += p.y;
				center_of_projected_cloud_z += p.z;
			    }

			    marker_array.markers.push_back(marker);
			    
// 			    center_of_projected_cloud.x= center_of_projected_cloud_x/all_projected_views.at(view_i)->points.size();
// 			    center_of_projected_cloud.y= center_of_projected_cloud_y/all_projected_views.at(view_i)->points.size();
// 			    center_of_projected_cloud.z= center_of_projected_cloud_z/all_projected_views.at(view_i)->points.size();

			    center_of_projected_cloud.x= 0.3;
			    center_of_projected_cloud.y= 0.3;
			    center_of_projected_cloud.z= 0.3;
			    
			}
			
			  /* _________________________________
			    |                                 |
			    |           Draw Plane            |
			    |_________________________________| */
			  if (1)
			  {
				  visualization_msgs::Marker marker;
				  marker.header.frame_id = tracker_frame_id;
				  marker.header.stamp = ros::Time();
			
				  marker.ns = "Plane";
				  marker.id = TID*33+view_i;
				  marker.type = visualization_msgs::Marker::CUBE;
				  marker.frame_locked = locked;
				  //if (finish)
				  //marker.action = visualization_msgs::Marker::DELETE;
				  //else
				  marker.action = visualization_msgs::Marker::ADD;
				  marker.lifetime = Duration(duration);
			
//  				  marker.pose.position =center_of_projected_cloud;
				  //compute_bounding_box_dimensions(all_projected_views.at(view_i), dimensions);
				  //ROS_INFO("Visuzlize box dimensions (x, y, z) = (%f, %f, %f) ", dimensions.x, dimensions.y, dimensions.z);
				  
				  if (view_i == 0)
				  {
					marker.pose.position.x = sign *  0.3;
					marker.scale.x = 0.0001;
					marker.scale.y = largest_side; 
					marker.scale.z = largest_side;
					
					dimensions.x = 0.0001; 
					dimensions.y = largest_side; 
					dimensions.z = largest_side;
				  }
				  else if (view_i ==1)
				  {
					marker.pose.position.y = sign *  0.3;
					marker.scale.x = largest_side; 
					marker.scale.y =  0.0001;
					marker.scale.z = largest_side; 
					
					dimensions.x = largest_side ; 
					dimensions.y = 0.0001;
					dimensions.z = largest_side; 
			
				  } 
				  else
				  {

				      marker.pose.position.z =  0.3;
				      marker.scale.x = largest_side; 
				      marker.scale.y = largest_side; 
				      marker.scale.z = 0.0001;
				      
				      dimensions.x = largest_side; 
				      dimensions.y = largest_side; 
				      dimensions.z = 0.0001;
				  }
				  			  
				  marker.color = cm->color(TID);		  
				  marker.color.a = 0.3;
						    
		
				  marker_array.markers.push_back(marker);
			  }
			  

				  
				  /* _________________________________
				    |                                 |
				    |         DRAW WIREFRAME          |
				    |_________________________________| */
				  if (1)
				  {
				    
					  visualization_msgs::Marker marker;
					  marker.header.frame_id = tracker_frame_id;
					  marker.header.stamp = ros::Time();
				
					  marker.ns = "wireframe";
					  marker.id = TID*44+view_i;
					  marker.frame_locked = locked;
					  marker.type = visualization_msgs::Marker::LINE_LIST;
					  //if (finish)
					  //marker.action = visualization_msgs::Marker::DELETE;
					  //else
					  marker.action = visualization_msgs::Marker::ADD;
					  marker.lifetime = Duration(duration);
							  
					  //marker.pose.position =center_of_projected_cloud;

					  //marker.pose = _tracked_object_msg.bounding_box.pose_stamped.pose;
				
					  marker.scale.x = 0.003; 
					  
				
					  marker.color.a = 0.5;
					  marker.color.r = 0.5;
					  marker.color.g = 0.5;
					  marker.color.b = 0.5;
					  //marker
					  if (finish)
					  {
						  marker.color.r = 0.1;
						  marker.color.g = 0.1;
						  marker.color.b = 0.1;
					  }

					  
					  
					double x = dimensions.x/2; 
					double y = dimensions.y/2; 
					double z = dimensions.z/2; 
// 					double x = largest_side/2; 
// 					double y = largest_side/2; 
// 					double z = largest_side/2; 
					

					    
					int k =number_of_bins;
					
					double interval_x = dimensions.x/k; 
					double interval_y = dimensions.y/k; 
					double interval_z = dimensions.z/k; 

// 					double interval_x = largest_side/k; 
// 					double interval_y = largest_side/k; 
// 					double interval_z = largest_side/k; 
				    
					if (view_i == 0)//for x
					{

					    marker.pose.position.x = sign * 0.3;

					  //ROS_INFO("YOZ WIREFRAME plane size (x, y, z) = (%f, %f, %f) ", x, y, z);   
					  // Page YOZ for Y axis
					      for (int i=0; i<= k ; i++)  
					      {	
						    p.x = x; p.y = -y + i*interval_y; p.z = -z; 
						    marker.points.push_back(p);
						    p.x = x; p.y = -y + i*interval_y ; p.z =  z; 
						    marker.points.push_back(p);	
					      }
					      
					      // Page YOZ for Z axis
					      for (int i=0; i<= k ; i++)  
					      {
						    p.x = x; p.y = y; p.z = -z + i*interval_z; 
						    marker.points.push_back(p);
						    p.x = x; p.y = -y ; p.z = -z + i*interval_z; ; 
						    marker.points.push_back(p);
					      }
					      marker_array.markers.push_back(marker);
					      
					}
					
					else if (view_i == 1)//for y
					{
					  //ROS_INFO("XOZ WIREFRAME plane size (x, y, z) = (%f, %f, %f) ", x, y, z);   
					    marker.pose.position.y = sign * 0.3;

					      // Page XOZ for X axis
					      for (int i=0; i<= k ; i++)  
					      {	
						    p.x = -x+ i*interval_x; p.y = y; p.z = -z; 
						    marker.points.push_back(p);
						    p.x = -x+ i*interval_x; p.y = y ; p.z =  z; 
						    marker.points.push_back(p);	
					      }
					      
					      // Page XOZ for Z axis
					      for (int i=0; i<= k ; i++)  
					      {
						    p.x = x; p.y = y; p.z = -z + i*interval_z; 
						    marker.points.push_back(p);
						    p.x = -x; p.y = y ; p.z = -z + i*interval_z; ; 
						    marker.points.push_back(p);
					      }
					      marker_array.markers.push_back(marker);

					}
					
					else //for z
					{
						marker.pose.position.z = 0.3;

					  //ROS_INFO("XOY WIREFRAME plane size (x, y, z) = (%f, %f, %f) ", x, y, z);   
					      // Page XOY for X axis
					      for (int i=0; i<= k ; i++)  
					      {	
						    p.x = -x+ i*interval_x; p.y = -y; p.z = z; 
						    marker.points.push_back(p);
						    p.x = -x+ i*interval_x; p.y = y ; p.z = z; 
						    marker.points.push_back(p);	
					      }
					      
					      // Page XOY for Y axis
					      for (int i=0; i<= k ; i++)  
					      {	
						    p.x = -x; p.y = -y + i*interval_y; p.z = z; 
						    marker.points.push_back(p);
						    p.x = x; p.y = -y + i*interval_y ; p.z = z; 
						    marker.points.push_back(p);	
					      }
					      marker_array.markers.push_back(marker);
					      
					}
	      		      } 
	      		     

				
	      		     
				    /* _______________________
				    |                         |
				    |         histogram       |
				    |_________________________| */
				  if (1)
				  {
				    
				        visualization_msgs::Marker marker;
					//marker.header.frame_id = object_frame_id;
					marker.header.stamp = ros::Time();		
					marker.frame_locked = locked;
					marker.header.frame_id = tracker_frame_id;

					marker.ns = "histogram of projected views";
					marker.id = TID*55+view_i;
					marker.type = visualization_msgs::Marker::POINTS;
					marker.lifetime = Duration(duration);
					marker.action = visualization_msgs::Marker::ADD;
// 					marker.pose.position.x = 0; marker.pose.position.y = 0;	marker.pose.position.z = 0;
// 					marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
					marker.color = cm->color(TID);
					marker.scale.x = 0.01; marker.scale.y = 0.01; marker.scale.z = 0.01; marker.color.a = 1;
					marker.color.r = 0.0; marker.color.g = 0.0;	marker.color.b = 1.0; marker.color.a = 1.0; 
										  
					  
					double x = dimensions.x/2; 
					double y = dimensions.y/2; 
					double z = dimensions.z/2; 
					    
					int k =number_of_bins;
					
					double interval_x = dimensions.x/k; 
					double interval_y = dimensions.y/k; 
					double interval_z = dimensions.z/k; 

// 					double interval_x = largest_side/k; 
// 					double interval_y = largest_side/k; 
// 					double interval_z = largest_side/k; 
				    
					if (view_i == 0)//for x
					{
					  //ROS_INFO("YOZ histogram");   
					  for (int i=0; i<all_projected_views.at(view_i)-> points.size(); i++)
					  {
						geometry_msgs::Point p;
						//p.x = all_projected_views.at(view_i)->points.at(i).x + x;
						// 1/2 dimention is added to the real data to put all the data in positive scale between [0,2y][0-2z]
						p.y = all_projected_views.at(view_i)->points.at(i).y + sign * y;
						//p.y = all_projected_views.at(view_i)->points.at(i).y + y;
						p.z = all_projected_views.at(view_i)->points.at(i).z + z;
						//ROS_INFO("YOZ: P(y).(%i)= %f --- dy= %f", all_projected_views.at(view_i)->points.at(i).y, y );
						//ROS_INFO("(trunc(p.y / interval_y)= %f and (trunc(p.z / interval_z) =%f", trunc(p.y / interval_y), trunc(p.z / interval_z));
						if ((trunc(p.y / interval_y) == 0.0) and ((trunc(p.z / interval_z) == 0.0)))
						{
						    geometry_msgs::Point p;
						    p.x=all_projected_views.at(view_i)->points.at(i).x;
						    p.y=all_projected_views.at(view_i)->points.at(i).y;
						    p.z=all_projected_views.at(view_i)->points.at(i).z;
						    marker.points.push_back(p);
			
// 						    p.x=all_projected_views.at(view_i)->points.at(i).x;
// 						    p.y= - all_projected_views.at(view_i)->points.at(i).y;
// 						    p.z=all_projected_views.at(view_i)->points.at(i).z;
// 						    marker.points.push_back(p);
// 						    
						}
					    }
					    marker_array.markers.push_back(marker);
					      
					}
					
					else if (view_i == 1)//for y
					{
					  //ROS_INFO("XOZ histogram");   
					  for (int i=0; i<all_projected_views.at(view_i)-> points.size(); i++)
					  {					    
						geometry_msgs::Point p;
						// 1/2 dimention is added to the real data to put all the data in positive scale between [0,2x][0-2z]
						p.x = all_projected_views.at(view_i)->points.at(i).x + sign * x;
						//p.x = all_projected_views.at(view_i)->points.at(i).x + x;
						//p.y = all_projected_views.at(view_i)->points.at(i).y;
						p.z = all_projected_views.at(view_i)->points.at(i).z + z;
						//ROS_INFO("XOZ: P(x).(%i)= %f --- dx= %f", all_projected_views.at(view_i)->points.at(i).x, x );
						//ROS_INFO("(trunc(p.x / interval_x)= %f and (trunc(p.z / interval_z) =%f", trunc(p.x / interval_x), trunc(p.z / interval_z));
						
						if ((trunc(p.x / interval_x) == 0.0) and ((trunc(p.z / interval_z) == 0.0)))
						{
						    geometry_msgs::Point p;
						    p.x=all_projected_views.at(view_i)->points.at(i).x;
						    p.y=all_projected_views.at(view_i)->points.at(i).y;
						    p.z=all_projected_views.at(view_i)->points.at(i).z;
						    marker.points.push_back(p);
						    
// 						    p.x= - all_projected_views.at(view_i)->points.at(i).x;
// 						    p.y=all_projected_views.at(view_i)->points.at(i).y;
// 						    p.z=all_projected_views.at(view_i)->points.at(i).z;
// 						    marker.points.push_back(p);
						}
					    }
					    marker_array.markers.push_back(marker);

					}
					
					else //for z
					{
					  //ROS_INFO("XOY histogram");   					       
					  for (int i=0; i<all_projected_views.at(view_i)-> points.size(); i++)
					  {
						geometry_msgs::Point p;
						// 1/2 dimention is added to the real data to put all the data in positive scale between [0,2x][0-2y]
						p.x = all_projected_views.at(view_i)->points.at(i).x + sign * x;						
						p.y = all_projected_views.at(view_i)->points.at(i).y + sign * y;
						//p.x = all_projected_views.at(view_i)->points.at(i).x +  x;						
						//p.y = all_projected_views.at(view_i)->points.at(i).y +  y;
						//p.z = all_projected_views.at(view_i)->points.at(i).z + z;
						//ROS_INFO("XOY: P(x).(%i)= %f --- dx= %f", all_projected_views.at(view_i)->points.at(i).x, x );
						//ROS_INFO("(trunc(p.x / interval_x)= %f and (trunc(p.y / interval_y) =%f", trunc(p.x / interval_x), trunc(p.y / interval_y));
						if ((trunc(p.x / interval_x) == 0.0) and ((trunc(p.y / interval_y) == 0.0)))
						{
						    geometry_msgs::Point p;
						    p.x=all_projected_views.at(view_i)->points.at(i).x;
						    p.y=all_projected_views.at(view_i)->points.at(i).y;
						    p.z=all_projected_views.at(view_i)->points.at(i).z;
						    marker.points.push_back(p);
						    
// 						    p.x= - all_projected_views.at(view_i)->points.at(i).x;
// 						    p.y= - all_projected_views.at(view_i)->points.at(i).y;
// 						    p.z=all_projected_views.at(view_i)->points.at(i).z;
// 						    marker.points.push_back(p);
						}
					    }
					    marker_array.markers.push_back(marker);
					      
					}
	      		      } 

	      		      
	      		      /* _________________________________
				|                                 |
				|         DRAW TEXT INFO          |
				|_________________________________| */
				if (1)
				{
					
					visualization_msgs::Marker marker;
					//marker.header.frame_id = object_frame_id;
					marker.header.stamp = ros::Time();		
					marker.frame_locked = locked;
					marker.header.frame_id = tracker_frame_id;

					//marker.pose.position =center_of_boundingbox_cloud;

					marker.ns = "sign";
					marker.id = TID*10+view_i;
					marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					marker.lifetime = Duration(duration);
					marker.action = visualization_msgs::Marker::ADD;
					//marker.pose.position.x = 0 + marker.pose.position.x*0.9 + 0;
					//marker.pose.position.y = 0 + marker.pose.position.y*0.9 + 0.1;
					marker.pose.position.z = largest_side + 0.15;
					marker.scale.z = 0.02; 
					marker.color.r  = 1; marker.color.g  = 0; marker.color.b  = 0; marker.color.a = 1;
					marker.text = "Sign = " + boost::lexical_cast<std::string>(sign);
					marker_array.markers.push_back(marker);
				}

				if (1)
				{
					
// 					visualization_msgs::Marker marker;
// 					//marker.header.frame_id = object_frame_id;
// 					marker.header.stamp = ros::Time();		
// 					marker.frame_locked = locked;
// 					marker.header.frame_id = tracker_frame_id;
// 
// 					//marker.pose.position =center_of_projected_cloud;
// 
// 					marker.ns = "entropy";
// 					marker.id = TID*100+view_i;
// 					marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
// 					marker.lifetime = Duration(duration);
// 					marker.action = visualization_msgs::Marker::ADD;
// 					//marker.pose.position.x = 0 + marker.pose.position.x*0.9 + 0;
// 					//marker.pose.position.y = 0 + marker.pose.position.y*0.9 + 0.1;
// 					marker.pose.position.z = largest_side + 0.25;
// 					marker.scale.z = 0.02; 
// 					marker.color.r  = 1; marker.color.g  = 0; marker.color.b  = 0; marker.color.a = 1;
// 					marker.text = "H(YoZ) = " + boost::lexical_cast<std::string>(view_point_entropy.at(0))+
// 						      "\nH(XoZ) = " + boost::lexical_cast<std::string>(view_point_entropy.at(1))+
// 						      "\nH(XoY) = " + boost::lexical_cast<std::string>(view_point_entropy.at(2));
// 					marker_array.markers.push_back(marker);
				}

				if (1)
				{
					
// 					visualization_msgs::Marker marker;
// 					//marker.header.frame_id = object_frame_id;
// 					marker.header.stamp = ros::Time();		
// 					marker.frame_locked = locked;
// 					marker.header.frame_id = tracker_frame_id;
// 
// 					//marker.pose.position =center_of_projected_cloud;
// 
// 					marker.ns = "sorted projections";
// 					marker.id = TID*200+view_i;
// 					marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
// 					marker.lifetime = Duration(duration);
// 					marker.action = visualization_msgs::Marker::ADD;
// 					//marker.pose.position.x = 0 + marker.pose.position.x*0.9 + 0;
// 					//marker.pose.position.y = 0 + marker.pose.position.y*0.9 + 0.1;
// 					marker.pose.position.z = largest_side + 0.3;
// 					marker.scale.z = 0.02; 
// 					marker.color.r  = 0; marker.color.g  = 0; marker.color.b  = 0; marker.color.a = 1;
// 					marker.text = std_name_of_sorted_projected_plane;
// 					marker_array.markers.push_back(marker);
				}

				
				
			      /* _______________________________
			      |                                 |
			      |           DRAW XYZ AXES         |
			      |_________________________________| */
				if (1)
				{	
					visualization_msgs::Marker marker;
					double axis_dimension = sign * 0.2;
					marker.header.frame_id = tracker_frame_id;
					marker.header.stamp = ros::Time();

					marker.frame_locked = locked;
					marker.type = visualization_msgs::Marker::LINE_STRIP;
					marker.action = visualization_msgs::Marker::ADD;
					marker.lifetime = Duration(duration);

					//marker.pose.position =center_of_projected_cloud;

					marker.scale.x = 0.01; marker.scale.y = 0.5; marker.scale.z = 4;

					//X axis
					marker.ns = "axes_x";
					marker.id = TID*1780+view_i;;
					marker.color.r = 1.0; marker.color.g = 0.0;	marker.color.b = 0.0; marker.color.a = 1.0; //red color
					marker.points.erase(marker.points.begin(), marker.points.end());
					p.x = 0; p.y = 0; p.z = 0; 
					marker.points.push_back(p);
					p.x = 1 * axis_dimension; p.y = 0; p.z = 0; 
					marker.points.push_back(p);
					marker_array.markers.push_back(marker);

					//Y axis
					marker.ns = "axes_y";
					marker.id = TID*10+view_i;
					marker.color.r = 0.0; marker.color.g = 1.0;	marker.color.b = 0.0; marker.color.a = 1.0; //green color
					marker.points.erase(marker.points.begin(), marker.points.end());
					p.x = 0; p.y = 0; p.z = 0; 
					marker.points.push_back(p);
					p.x = 0; p.y = 1 * axis_dimension; p.z = 0; 
					marker.points.push_back(p);
					marker_array.markers.push_back(marker);

					//Z axis
					marker.ns = "axes_z";
					marker.id = TID*10+view_i;
					marker.color.r = 0.0; marker.color.g = 0.0;	marker.color.b = 1.0; marker.color.a = 1.0; //blue color
					marker.points.erase(marker.points.begin(), marker.points.end());
					p.x = 0; p.y = 0; p.z = 0; 
					marker.points.push_back(p);
					
					if (sign > 0 )
					{
					  p.x = 0; p.y = 0; p.z = 1 * axis_dimension;  
					}
					else 
					{
					  p.x = 0; p.y = 0; p.z = 1 * -axis_dimension;
					}
					 
					marker.points.push_back(p);
					marker_array.markers.push_back(marker);
					ROS_INFO("axes added...");
				}
				
				}
				

		      neat_marker_publisher.publish(marker_array);
		      return 1;
		  }
		
	    	    
	    	    
	    	    
	    	    
	    template <typename T>
	    int set_neat_visualization_marker_array_object_descriptor( boost::shared_ptr<pcl::PointCloud<T> > all_projected_views, string object_frame_id, unsigned int TID )
	      {

		//STEP 1: need to get the position of the object so we can draw
		//text nearby
		//   tf::StampedTransform stf; //the transform
		//   //std::string tracker_frame_id =  msg->header.frame_id;
		//   std::string tracker_frame_id = "/perception/pipeline" + boost::lexical_cast<std::string>(msg->track_id) + "/tracker";

		std::string tracker_frame_id = "/perception/pipeline" + boost::lexical_cast<std::string>(TID) + "/tracker";
// 		ROS_INFO ("set_neat_visualization_marker_array_object_descriptor: object_frame_id = %s", object_frame_id.c_str());
		
		visualization_msgs::MarkerArray marker_array; 
// 		geometry_msgs::Point p;
		double duration = 5;
		bool locked = true;
		bool finish= true;
		float center_of_projected_cloud_x=0 , center_of_projected_cloud_y=0 , center_of_projected_cloud_z=0 ;
		geometry_msgs::Point center_of_projected_cloud;
		geometry_msgs::Vector3 dimensions;
		geometry_msgs::Point p;

		/* _________________________________
	      	   |                                 |
	      	   |   Draw Projected Point Cloud    |
	      	   |_________________________________| */
	      	if (1)
	      	{
		      visualization_msgs::Marker marker;
		      //marker.header.frame_id = object_frame_id;
		      marker.header.stamp = ros::Time();		
		      marker.frame_locked = locked;
		      marker.header.frame_id = tracker_frame_id;

		      marker.ns = "projected views";
		      marker.id = TID;
		      marker.type = visualization_msgs::Marker::POINTS;
		      marker.lifetime = Duration(duration);
		      marker.action = visualization_msgs::Marker::ADD;
		      marker.pose.position.x = 0;	marker.pose.position.y = 0;	marker.pose.position.z = 0;
		      marker.pose.orientation.x = 0.0; marker.pose.orientation.y = 0.0; marker.pose.orientation.z = 0.0; marker.pose.orientation.w = 1.0;
		      marker.color = cm->color(TID);
		      marker.scale.x = 0.005; marker.scale.y = 0.005; marker.scale.z = 0.005; marker.color.a = 1;
		      //marker.color.r = 1.0; marker.color.g = 1.0;	marker.color.b = 1.0; marker.color.a = 1.0; 
		      
		      //marker.points.erase(marker.points.begin(), marker.points.end());
		      geometry_msgs::Point p;

		      for (size_t i=0; i<all_projected_views->points.size(); i++)
		      {
			  p.x = all_projected_views->points.at(i).x;
			  p.y = all_projected_views->points.at(i).y;
			  p.z = all_projected_views->points.at(i).z;
			  marker.points.push_back(p);
			  center_of_projected_cloud_x += p.x;
			  center_of_projected_cloud_y += p.y;
			  center_of_projected_cloud_z += p.z;
		      }
		      center_of_projected_cloud.x= center_of_projected_cloud_x/all_projected_views->points.size();
		      center_of_projected_cloud.y= center_of_projected_cloud_y/all_projected_views->points.size();
		      center_of_projected_cloud.z= center_of_projected_cloud_z/all_projected_views->points.size();
		      
		      marker_array.markers.push_back(marker);
		}
	      
		      /* _________________________________
			|                                 |
			|             DRAW BBOX           |
			|_________________________________| */
		      if (1)
		      {
			      visualization_msgs::Marker marker;
			      marker.header.frame_id = tracker_frame_id;
			      marker.header.stamp = ros::Time();
		    
			      marker.ns = "boundingbox";
			      marker.id = TID;
			      marker.type = visualization_msgs::Marker::CUBE;
			      marker.frame_locked = locked;
			      //if (finish)
			      //marker.action = visualization_msgs::Marker::DELETE;
			      //else
			      marker.action = visualization_msgs::Marker::ADD;
			      marker.lifetime = Duration(5);
		    
			      
			      marker.pose.position =center_of_projected_cloud;
			      compute_bounding_box_dimensions(all_projected_views, dimensions);
			      ROS_INFO("box dimensions (x, y, z) = (%f, %f, %f) ", dimensions.x, dimensions.y, dimensions.z);
		      
			      marker.scale.x = dimensions.x+0.0; 
			      marker.scale.y = dimensions.y+0.2; 
			      marker.scale.z = dimensions.z+0.02; 
		    
			      marker.color = cm->color(TID);
						
			      marker.color.a = 0.3;
			      marker.color.r = 0.0;
			      marker.color.g = 0.5;
			      marker.color.b = 0.5;
			      if (finish)
			      {
				      marker.color.r = 0.9;
				      marker.color.g = 0.0;
				      marker.color.b = 0.0;
			      }
		    
			      marker_array.markers.push_back(marker);
		      }
		      /* _________________________________
			|                                 |
			|             DRAW WIREFRAME      |
			|_________________________________| */
		      if (1)
		      {
			
			      visualization_msgs::Marker marker;
			      marker.header.frame_id = tracker_frame_id;
			      marker.header.stamp = ros::Time();
		    
			      marker.ns = "wireframe";
			      marker.id = TID;
			      marker.frame_locked = locked;
			      marker.type = visualization_msgs::Marker::LINE_LIST;
			      //if (finish)
			      //marker.action = visualization_msgs::Marker::DELETE;
			      //else
			      marker.action = visualization_msgs::Marker::ADD;
			      marker.lifetime = Duration(5);
		    			      
			      marker.pose.position =center_of_projected_cloud;

			      //marker.pose = _tracked_object_msg.bounding_box.pose_stamped.pose;
		    
			      marker.scale.x = 0.005; 
			      double x = dimensions.x/2; 
			      double y = dimensions.y/2; 
			      double z = dimensions.z/2; 
		    
			      marker.color.a = 0.5;
			      marker.color.r = 0.5;
			      marker.color.g = 0.5;
			      marker.color.b = 0.5;
			      //marker
			      if (finish)
			      {
				      marker.color.r = 0.1;
				      marker.color.g = 0.1;
				      marker.color.b = 0.1;
			      }

			      //outside lines
			      			      
			      p.x =  x; p.y =  y; p.z = -z; marker.points.push_back(p);
			      p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
			      p.x =  x; p.y =  -y; p.z = -z; marker.points.push_back(p);
			      p.x =  x; p.y =  -y; p.z =  z; marker.points.push_back(p);
			      p.x =  x; p.y =  -y; p.z = z; marker.points.push_back(p);
			      p.x =  x; p.y =  y; p.z =  z; marker.points.push_back(p);
			      p.x =  x; p.y =  -y; p.z = -z; marker.points.push_back(p);
			      p.x =  x; p.y =  y; p.z =  -z; marker.points.push_back(p);
			      
			      
			      //inside	lines		      
			      p.x =  x; p.y =  y/2; p.z = -z; marker.points.push_back(p);
			      p.x =  x; p.y =  y/2; p.z =  z; marker.points.push_back(p);			      
			      p.x =  x; p.y =  0 ; p.z = -z; marker.points.push_back(p);
			      p.x =  x; p.y =  0; p.z =  z; marker.points.push_back(p);
			      p.x =  x; p.y =  -y/2; p.z = -z; marker.points.push_back(p);
			      p.x =  x; p.y =  -y/2; p.z =  z; marker.points.push_back(p);
			      
			    
			      p.x =  x; p.y =  -y; p.z = z/2; marker.points.push_back(p);
			      p.x =  x; p.y =  y; p.z =  z/2; marker.points.push_back(p);	
			      
			      p.x =  x; p.y =  -y; p.z = 0; marker.points.push_back(p);
			      p.x =  x; p.y =  y; p.z =  0; marker.points.push_back(p);	
			     
			      p.x =  x; p.y =  -y; p.z = -z/2; marker.points.push_back(p);
			      p.x =  x; p.y =  y; p.z =  -z/2; marker.points.push_back(p);	
			      
			      
			      
			      marker_array.markers.push_back(marker);
		      }
	      
	      	

		      neat_marker_publisher.publish(marker_array);
		      return 1;


            void Time();
		  }
//            void ROS_INFO(const char* arg1, int adaptive_support_lenght);
            //void ROS_INFO(const char* arg1, int adaptive_support_lenght);
			//  void rot_mat(int arg1);


    };

    
    
    class ObjectDescriptorDeepLearningNodelet: public ObjectDescriptorDeepLearning<pcl::PointXYZRGBA>{};
//    PLUGINLIB_DECLARE_CLASS(race_object_descriptor, ObjectDescriptorDeepLearningNodelet, race_object_descriptor::ObjectDescriptorDeepLearningNodelet, nodelet::Nodelet);
   PLUGINLIB_EXPORT_CLASS(race_object_descriptor_deep_learning::ObjectDescriptorDeepLearningNodelet, nodelet::Nodelet);
}//end feature_extraction namespace
#endif



