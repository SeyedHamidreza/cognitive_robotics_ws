#ifndef _NODE_OBJECT_CONCEPTUALIZER_CPP_
#define _NODE_OBJECT_CONCEPTUALIZER_CPP_
#endif

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

#include <ros/ros.h>
#include <object_conceptualizer/object_conceptualizer.h>

using namespace race_object_conceptualizer;

//node can be tested by running
//rostopic  pub /test_msg std_msgs/String "data: 'hello'"

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */


int main (int argc, char** argv)
{
	ros::init(argc, argv, "node_object_conceptualizer"); // Initialize ROS coms
	ros::NodeHandle* n;
	n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	ObjectConceptualizer<pcl::PointXYZRGBA> object_conceptualizer(n); //initialize the class
		
	ros::Time t = ros::Time::now();
	ros::Rate loop_rate(2);
	ros::spin();

	return 1;
}

