
#include <ros/ros.h>
#include <race_perception_utils/print.h>
#include <tf/transform_broadcaster.h>

using namespace race_perception_utils;
using namespace std;
using namespace tf;

boost::shared_ptr<TransformBroadcaster> _br; //a transform broadcaster

//<arg name="pi/2" value="1.5707963267948966" />
//<arg name="optical_rotate" value="0 0 0 -$(arg pi/2) 0 -$(arg pi/2)" />

//<node pkg="tf" type="static_transform_publisher" name="$(arg camera)_base_link"
//args="0 -0.02  0 0 0 0  /$(arg camera)_link /$(arg camera)_depth_frame 300" />  
//
//<node pkg="tf" type="static_transform_publisher" name="$(arg camera)_base_link1"
//args="0 -0.045 0 0 0 0  /$(arg camera)_link /$(arg camera)_rgb_frame 300" />  
//
//<node pkg="tf" type="static_transform_publisher" name="$(arg camera)_base_link2"
//args="$(arg optical_rotate) /$(arg camera)_depth_frame /$(arg camera)_depth_optical_frame  300" />  
//
//<node pkg="tf" type="static_transform_publisher" name="$(arg camera)_base_link3"
//args="$(arg optical_rotate) /$(arg camera)_rgb_frame /$(arg camera)_rgb_optical_frame 300" />  



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "kinect_static_tf"); // Initialize ROS coms
	ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	//Initialize tf stuff
	_br = (boost::shared_ptr<TransformBroadcaster>) new TransformBroadcaster;

	//define all tfs
	Quaternion Q; 
	tf::Transform trf1, trf2, trf3, trf4;

	trf1.setOrigin(Vector3(0, -0.02, 0)); 
	Q.setRPY(0,0,0); trf1.setRotation(Q); 

	trf2.setOrigin(Vector3(0, -0.045, 0)); 
	Q.setRPY(0,0,0); trf2.setRotation(Q); 

	trf3.setOrigin(Vector3(0, 0, 0)); 
	Q.setRPY(-M_PI/2, 0, -M_PI/2); trf3.setRotation(Q); 

	trf4.setOrigin(Vector3(0, 0, 0)); 
	Q.setRPY(-M_PI/2, 0, -M_PI/2); trf4.setRotation(Q); 

	PrettyPrint pp;
	pp.printInitialization();

	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		loop_rate.sleep(); //sleep
		ros::spinOnce(); // Handle ROS events

		ros::Time t = ros::Time::now();
		_br->sendTransform(StampedTransform(trf1, t+ros::Duration(1), "/camera_link", "/camera_depth_frame"));
		_br->sendTransform(StampedTransform(trf2, t+ros::Duration(1), "/camera_link", "/camera_rgb_frame"));
		_br->sendTransform(StampedTransform(trf3, t+ros::Duration(1), "/camera_depth_frame", "/camera_depth_optical_frame"));
		_br->sendTransform(StampedTransform(trf4, t+ros::Duration(1), "/camera_rgb_frame", "/camera_rgb_optical_frame"));


	}

	return 1;
}
