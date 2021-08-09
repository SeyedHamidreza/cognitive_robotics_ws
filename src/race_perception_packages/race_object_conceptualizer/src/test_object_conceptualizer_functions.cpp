/* _________________________________
  |                                 |
  |           INCLUDES              |
  |_________________________________| */
//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <vector>

//ros includes 
#include <ros/ros.h>
#include <ros/package.h>


//pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

// ROS includes
#include <sensor_msgs/PointCloud2.h>

//package includes
#include <feature_extraction/spin_image.h>
#include <race_3d_object_tracking/TrackedObjectPointCloud.h>
#include <object_conceptualizer/object_conceptualization.h>
#include <race_perception_utils/cycle.h>
#include <race_perception_utils/print.h>
/* _________________________________
  |                                 |
  |         Global variable         |
  |_________________________________| */

  
typedef pcl::PointXYZRGBA PointT;

using namespace pcl;
using namespace std;
using namespace ros;
using namespace race_perception_utils;


int main(int argc, char** argv)
{
	PrettyPrint pp;    
	ros::init (argc, argv, "Test_intraCategoryDistance_function");
	ros::NodeHandle nh2;
	ROS_INFO("\t\t[-]Test_intraCategoryDistance_function");
	std::vector< vector <SITOV> > category_instances; 
	
	std::string path;
	path  = ros::package::getPath("race_feature_extraction") + "/pcd/obj0.pcd";

	boost::shared_ptr<PointCloud<PointT> > msg_pc (new PointCloud<PointT>);
	if (io::loadPCDFile <PointXYZRGBA> (path.c_str(), *msg_pc) == -1)
	{	
		ROS_ERROR("\t\t[-]-Could not read given object %s :",path.c_str());
		return(0);
	}
	else
	{
		ROS_INFO("\t\t[1]-Loaded a point cloud: %s", path.c_str());
	}
	
	//Declare a boost share ptr to the SITOV msg
	boost::shared_ptr< vector <SITOV> > msg1;
	msg1 = (boost::shared_ptr< vector <SITOV> >) new (vector <SITOV>);
	//Call the library function for estimateSpinImages
	estimateSpinImages(msg_pc, 
			0.01 /*downsampling_voxel_size*/, 
			0.05 /*normal_estimation_radius*/,
			8    /*spin_image_width*/,
			0.0 /*spin_image_cos_angle*/,
			1   /*spin_image_minimum_neighbor_density*/,
			0.2 /*spin_image_support_lenght*/,
			msg1,
			0
			);
				
	ROS_INFO("\t\t[-]-There are %ld keypoints", msg1->size());
	if (msg1->size()>0)
	{	
	    ROS_INFO("\t\t[-]-Spin image of keypoint 0 has %ld elements", msg1->at(0).spin_image.size());
	}
	 	
	category_instances.push_back(*msg1);

///////////////////////////////////////////////////////
	std::string path2;
	path2  = ros::package::getPath("race_feature_extraction") + "/pcd/obj1.pcd";

	boost::shared_ptr<PointCloud<PointT> > msg_pc2 (new PointCloud<PointT>);
	if (io::loadPCDFile <PointXYZRGBA> (path2.c_str(), *msg_pc2) == -1)
	{	
		ROS_ERROR("\t\t[-]-Could not read given object %s :",path2.c_str());
		return(0);
	}
	else
	{
		ROS_INFO("\t\t[2]-Loaded a point cloud: %s ", path2.c_str());
	}
	
	//Declare a boost share ptr to the SITOV msg
	boost::shared_ptr< vector <SITOV> > msg2;
	msg2 = (boost::shared_ptr< vector <SITOV> >) new (vector <SITOV>);
	//Call the library function for estimateSpinImages
	estimateSpinImages(msg_pc2, 
			0.01 /*downsampling_voxel_size*/, 
			0.05 /*normal_estimation_radius*/,
			8    /*spin_image_width*/,
			0.0 /*spin_image_cos_angle*/,
			1   /*spin_image_minimum_neighbor_density*/,
			0.2 /*spin_image_support_lenght*/,
			msg2,
			0
			);
				
	ROS_INFO("\t\t[-]-There are %ld keypoints", msg2->size());
	if (msg2->size()>0)
	{	
	    ROS_INFO("\t\t[-]-Spin image of keypoint 0 has %ld elements", msg2->at(0).spin_image.size());
	}
	    	
	category_instances.push_back(*msg2);

////////////////////////////////////////////////////////////	
	std::string path3;
	path3  = ros::package::getPath("race_feature_extraction") + "/pcd/obj2.pcd";

	boost::shared_ptr<PointCloud<PointT> > msg_pc3 (new PointCloud<PointT>);
	
	if (io::loadPCDFile <PointXYZRGBA> (path3.c_str(), *msg_pc3) == -1)
	{	
		ROS_ERROR("\t\t[-]-Could not read given object %s :",path3.c_str());
		return(0);
	}
	else
	{
		ROS_INFO("\t\t[2]-Loaded a point cloud: %s", path3.c_str());
	}
	
	//Declare a boost share ptr to the SITOV msg
	boost::shared_ptr< vector <SITOV> > msg3;
	msg3 = (boost::shared_ptr< vector <SITOV> >) new (vector <SITOV>);
	//Call the library function for estimateSpinImages
	estimateSpinImages(msg_pc3, 
			0.01 /*downsampling_voxel_size*/, 
			0.05 /*normal_estimation_radius*/,
			8    /*spin_image_width*/,
			0.0 /*spin_image_cos_angle*/,
			1   /*spin_image_minimum_neighbor_density*/,
			0.2 /*spin_image_support_lenght*/,
			msg3,
			0
			);
				
	ROS_INFO("\t\t[-]-There are %ld keypoints", msg3->size());
	if (msg3->size()>0)
	{	
	    ROS_INFO("\t\t[-]-Spin image of keypoint 0 has %ld elements", msg3->at(0).spin_image.size());
	}
	    	
	category_instances.push_back(*msg3);
//////////////////////////////////////////////////////////////////////
	std::string pathTest;
	pathTest  = ros::package::getPath("race_feature_extraction") + "/pcd/obj4.pcd";

	boost::shared_ptr<PointCloud<PointT> > msg_pcTest (new PointCloud<PointT>);
	
	if (io::loadPCDFile <PointXYZRGBA> (pathTest.c_str(), *msg_pcTest) == -1)
	{	
		ROS_ERROR("\t\t[-]-Could not read given object: %s",pathTest.c_str());
		return(0);
	}
	else
	{
		ROS_INFO("\t\t[2]-Loaded a point cloud: %s", pathTest.c_str());
	}
	
	//Declare a boost share ptr to the SITOV msg
	boost::shared_ptr< vector <SITOV> > msgTest;
	msgTest = (boost::shared_ptr< vector <SITOV> >) new (vector <SITOV>);
	//Call the library function for estimateSpinImages
	estimateSpinImages(msg_pcTest, 
			0.01 /*downsampling_voxel_size*/, 
			0.05 /*normal_estimation_radius*/,
			8    /*spin_image_width*/,
			0.0 /*spin_image_cos_angle*/,
			1   /*spin_image_minimum_neighbor_density*/,
			0.2 /*spin_image_support_lenght*/,
			msgTest,
			0
			);
				
	ROS_INFO("\t\t[-]-There are %ld keypoints", msgTest->size());
	if (msgTest->size()>0)
	{	
	    ROS_INFO("\t\t[-]-Spin image of keypoint 0 has %ld elements", msgTest->at(0).spin_image.size());
	}
	
	float objectToCategoryDistance;
	int best_matched_index;
	objectCategoryDistance(*msgTest,category_instances,objectToCategoryDistance,best_matched_index, pp);//D(O,C)
	ROS_INFO("\t\t[-]-test_objectCategoryDistance, objectCategoryDistance is: %f , Best_matched_index is : %i", objectToCategoryDistance, best_matched_index );

	float ICD =0;
	intraCategoryDistance(category_instances,ICD, pp);
	ROS_INFO("\t\t[-]-test_ICD value after calling function is: %f ", ICD);
	
	float normalizedDistance=0;
	normalizedObjectCategoryDistance(objectToCategoryDistance,ICD,normalizedDistance, pp);
	ROS_INFO("\t\t[-]-test_normalizedObjectCategoryDistance, normalizedObjectCategoryDistance is: %f ", normalizedDistance);

	// Spin
 	ros::spin ();

	return 1;
}
