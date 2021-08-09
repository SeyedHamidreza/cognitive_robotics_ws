/**
 * @file
 * @brief 
 */
#ifndef _TEST_WAIT_FOR_TF_CPP_
#define _TEST_WAIT_FOR_TF_CPP_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//ros includes
#include <ros/ros.h>
#include <tf/transform_listener.h>

//rviz includes


//race ua includes
#include <race_perception_utils/print.h>
#include <race_perception_utils/tf_wrapper.h>

/* _________________________________
  |                                 |
  |           Namespaces            |
  |_________________________________| */

using namespace std;
using namespace tf;
using namespace ros;
using namespace race_perception_utils;

//Global vars
boost::shared_ptr<tf::TransformListener> p_tf_listener;
string _name;


int main (int argc, char** argv)
{
    PrettyPrint pp;
    ros::init(argc, argv, "test_wait_for_tf"); // Initialize ROS coms

    ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

    //get node name
    _name = n->getNamespace();

    string ns = (ros::this_node::getNamespace()).substr(1); //to get the namespace with a single / at the beggining

	//init listener
	p_tf_listener = (boost::shared_ptr<tf::TransformListener>) new tf::TransformListener;

    std::string target = "/map";
    std::string source = "/perception/tabletop_segmentation/table";

    ROS_INFO("Starting to wait for tf between %s and %s", target.c_str(), source.c_str());


    //Wait for the first table transform
    if(!race_perception_utils::safe_tf_wait(p_tf_listener, "/map", "/perception/tabletop_segmentation/table", 10, _name))
    {
        pp.info(std::ostringstream().flush() << "Could not get table transform after waiting forever. Aborting"); 
	    pp.printInitialization();
        exit(1);
    }
	ros::Duration(0.4).sleep();

    ROS_INFO("Finished to wait for tf");
    return 1;
}

#endif
