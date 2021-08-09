
#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

#include <ros/ros.h>
//#include <race_perception_db/perception_db.h>
#include "perception_db_nodelet.h"

using namespace race_perception_db;

//node can be tested by running
//rostopic  pub /test_msg std_msgs/String "data: 'hello'"

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */


int main (int argc, char** argv)
{
	ros::init(argc, argv, "perception_db_node"); // Initialize ROS coms
	ros::NodeHandle* n;
	n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle
	PerceptionDBNodelet pdb(n, false, true); //initialize the class

	ros::Time t = ros::Time::now();
	ros::Rate loop_rate(10);
	ros::spin();

	return 1;
}

