#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>


#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>
#include <race_perception_msgs/perception_msgs.h>

#include <race_perception_utils/print.h>
#include <race_perception_utils/cycle.h>
#include <race_perception_utils/MeasurePDB.h>

using namespace race_perception_db;
using namespace race_perception_msgs;
using namespace race_perception_utils;

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */

vector<string> vs;
vector<float> vf;
boost::shared_ptr<ros::Publisher> _p_publisher;


std::string _name="pdbreport";
PerceptionDB* _pdb; //initialize the class

MeasurePDB mc_msg;
	
void timer_callback(const ros::TimerEvent& input)
{

	mc_msg = _pdb->reportPDB();
	
    //ROS_INFO("rtov: %f, sitov: %f in callback ", mc_msg.rtov, mc_msg.sitov);
    //for (size_t i =0 ; i< report.size(); i++)
	//{
	//	ROS_INFO("%s", report[i].c_str());
	//}

	mc_msg.header.stamp = ros::Time::now();
    mc_msg.sitov /= 100.0;
    mc_msg.rtov /= 10.0;

    //mc_msg.pdb_size /= 1024;

	_p_publisher->publish(mc_msg);

}

void callback(const race_perception_utils::Cycle& msg)
{
	//mc_msg = _pdb->reportPDB();
    
    //vs.push_back("pctov");
	//vf.push_back(mc_msg.pctov);

  	//vs.push_back("sitov");
	//vf.push_back(mc_msg.sitov);

 
   	//vs.push_back("rtov");
	//vf.push_back(mc_msg.rtov);


	//vs.push_back("oc");
	//vf.push_back(mc_msg.oc);

	//vs.push_back("rrtov");
	//vf.push_back(mc_msg.rrtov);
    //ROS_INFO("rtov: %f, sitov: %f in callback ", mc_msg.rtov, mc_msg.sitov);
}


int main (int argc, char** argv)
{


	ros::init(argc, argv, _name); // Initialize ROS coms
	ros::NodeHandle* _p_nh;
	_p_nh = (ros::NodeHandle*) new ros::NodeHandle(_name); //The node handle
	_name = _p_nh->getNamespace();

	_pdb = race_perception_db::PerceptionDB::getPerceptionDB(_p_nh); //initialize the class
	
    //ros::Subscriber _subscriber = _p_nh->subscribe ("/perception/cycle", 10, callback);

    ros::Timer _timer = _p_nh->createTimer(ros::Duration(1.0), timer_callback);

	_p_publisher = (boost::shared_ptr<ros::Publisher>) new (ros::Publisher);
	*_p_publisher = _p_nh->advertise<race_perception_utils::MeasurePDB>("/perception/pdbreport", 100);

	PrettyPrint pp;
	pp.printInitialization();
	ros::spin();

	return 1;
}

