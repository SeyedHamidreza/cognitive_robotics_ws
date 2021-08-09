/**
 * @file
 * @brief A simple code to get pointing detection based on the tf information
 * provided by openni skeleton tracker.
 */
#ifndef _OBJEC_PERCEPTION_SYSTEM_MANAGER_CPP_
#define _OBJEC_PERCEPTION_SYSTEM_MANAGER_CPP_

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <race_perception_utils/print.h>

using namespace race_perception_utils;
using namespace std;

//bool service_callback(race_gesture_detection::GetPointingDirection::Request  &req, race_gesture_detection::GetPointingDirection::Response &res)
//{
	//PrettyPrint pp;
	//pp.info(std::ostringstream().flush() << "Received a GetPointingDirection client request"); 


	//pp.printCallback();
	//return true;
//}

int main (int argc, char** argv)
{

	ros::init(argc, argv, "pointing_detection"); // Initialize ROS coms

	ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	PrettyPrint pp;

	//get node name
	//_name = n->getNamespace();

	//init service
	//std::string _service_name = n->resolveName("object_perception_scene_manager");
	//ros::ServiceServer service = n->advertiseService(_service_name, service_callback);

	////init subscribers
	//string ns = (ros::this_node::getNamespace()).substr(1); //to get the namespace with a single / at the beggining
	//ros::Subscriber sub = n->subscribe(ns + "/demonstrator_detection/demonstrator_id", 1, demonstratorCallback);

	//Start program
	pp.printInitialization();

	ros::Rate loop_rate(20);
	loop_rate.sleep(); //sleep

    ROS_ERROR("STARTING OBJECT PERCEPTION");

    boost::thread perception_thread(system, "roslaunch race_perception_bringup bringup.launch use_pr2:=1");

    sleep(5);


    ROS_ERROR("FINISHING OBJECT PERCEPTION");
	boost::thread::terminate();
    //perception_thread.interrupt();
    //perception_thread.terminate();

	return 1;
}

#endif
