
#include <ros/ros.h>
#include <race_perception_utils/print.h>
#include <tf/transform_listener.h>

using namespace race_perception_utils;
using namespace std;

boost::shared_ptr<tf::TransformListener> p_tf_listener;


void callback(int num)
{
	PrettyPrint pp;
	pp.info(std::ostringstream().flush() << "Variable is " << num);
	pp.printCallback();
}


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_print"); // Initialize ROS coms
	ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	//here we instantiate a PrettyPrint object.
	//For an onInit function
	//PrettyPrint pp;

	//int var1 = 4;
	//string var2 = "/path_to_something";
	//pp.info(std::ostringstream().flush() << "This is a custom text with var1 = " << var1);
	//pp.info(std::ostringstream().flush() << "This is a custom text with var2 = " << var2);
	//pp.warn(std::ostringstream().flush() << "This is a warning");
	//pp.error(std::ostringstream().flush() << "This is an error");
	//pp.printInitialization();
	////just to show when the callback is called
	//callback(7);

	//init tf listener
	p_tf_listener = (boost::shared_ptr<tf::TransformListener>) new tf::TransformListener;


	tf::StampedTransform transf;

	//ros::Duration(1).sleep();
	ros::Rate loop_rate(20);
	bool flag=false;
	while (flag)
	{
		ros::Time now = ros::Time::now();
		if (now.toSec()!=0)
			flag=false;
		else
			loop_rate.sleep(); //sleep
	}


	while (ros::ok())
	{

		try{
		ros::Time now = ros::Time::now();
			ROS_INFO_STREAM("Time is " << now);
			p_tf_listener->waitForTransform("/base_footprint" , "/base_link", now, ros::Duration(3.0));
			p_tf_listener->lookupTransform("/base_footprint" , "/base_link", now, transf);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
		}

		loop_rate.sleep(); //sleep
		ros::spinOnce(); // Handle ROS events

	}
}
