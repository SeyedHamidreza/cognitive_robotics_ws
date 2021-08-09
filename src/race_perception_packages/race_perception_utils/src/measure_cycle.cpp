
#include <ros/ros.h>
#include <race_perception_utils/print.h>
#include <race_perception_utils/cycle.h>
#include <race_perception_utils/Cycle.h>
#include <race_perception_utils/MeasureCycle.h>

using namespace race_perception_utils;
using namespace std;

vector<string> vs;
vector<float> vf;
boost::shared_ptr<ros::Publisher> _p_publisher;

void timerCallback(const ros::TimerEvent& input)
{
	MeasureCycle mc_msg;

	for (size_t i=0; i<vs.size(); ++i)
	{
		if (vs[i]=="/perception/object_detection")
			mc_msg.object_detection = vf[i];
		else if (vs[i]=="/perception/tabletop_segmentation")
			mc_msg.tabletop_segmentation = vf[i];
		else if (vs[i]=="/perception/pipeline0/tracker")
			mc_msg.tracker0 = vf[i];
		else if (vs[i]=="/perception/pipeline1/tracker")
			mc_msg.tracker1 = vf[i];
		else if (vs[i]=="/perception/pipeline2/tracker")
			mc_msg.tracker2 = vf[i];
		else if (vs[i]=="/perception/pipeline3/tracker")
			mc_msg.tracker3 = vf[i];
		else if (vs[i]=="/perception/pipeline0/feature_extraction")
			mc_msg.fe0 = vf[i];
		else if (vs[i]=="/perception/pipeline1/feature_extraction")
			mc_msg.fe1 = vf[i];
		else if (vs[i]=="/perception/pipeline2/feature_extraction")
			mc_msg.fe2 = vf[i];
		else if (vs[i]=="/perception/pipeline3/feature_extraction")
			mc_msg.fe3 = vf[i];
		else if (vs[i]=="/perception/pipeline0/object_recognition")
			mc_msg.or0 = vf[i];
		else if (vs[i]=="/perception/pipeline1/object_recognition")
			mc_msg.or1 = vf[i];
		else if (vs[i]=="/perception/pipeline2/object_recognition")
			mc_msg.or2 = vf[i];
		else if (vs[i]=="/perception/pipeline3/object_recognition")
			mc_msg.or3 = vf[i];


	}

	mc_msg.header.stamp = ros::Time::now();
	_p_publisher->publish(mc_msg);
	
}

void callback(const race_perception_utils::Cycle& msg)
{
	//ROS_INFO("%s is at frequency %f", msg.name.c_str(), msg.frequency);
	bool found=false;

	for (size_t i=0; i<vs.size(); ++i)
	{
		if (msg.name == vs[i])
		{
			vf[i] = msg.frequency;
			found = true;
		}

		if (found) break;
	}

	if (found==false)
	{
		vs.push_back(msg.name);
		vf.push_back(msg.frequency);
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "measure_cycle"); // Initialize ROS coms
	ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	ros::Subscriber _subscriber = n->subscribe ("/perception/cycle", 10, callback);

	ros::Timer _timer = n->createTimer(ros::Duration(2), &timerCallback);

	_p_publisher = (boost::shared_ptr<ros::Publisher>) new (ros::Publisher);
	*_p_publisher = n->advertise<race_perception_utils::MeasureCycle>("/perception/mcycle", 100);

	PrettyPrint pp;
	pp.printInitialization();
	ros::spin();

return 1;
}
