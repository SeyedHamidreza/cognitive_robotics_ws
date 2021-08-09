#ifndef _NODE_OBJECT_DESCRIPTOR_CPP_
#define _NODE_OBJECT_DESCRIPTOR_CPP_
#endif

#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

#include <ros/ros.h>
#include <object_descriptor_deep_learning/object_descriptor_deep_learning.h>

using namespace race_object_descriptor_deep_learning;

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */

#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>

std::string _name;
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
    g_request_shutdown = 1;

    ROS_INFO("%s: Shutdown request received. Reason: [%s]", _name.c_str(),  "Received SIGINT");
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    int num_params = 0;
    if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
    if (num_params > 1)
    {
        std::string reason = params[1];
        ROS_WARN("%s: Shutdown request received. Reason: [%s]", _name.c_str(),  reason.c_str());
        g_request_shutdown = 1; // Set flag
    }

    result = ros::xmlrpc::responseInt(1, "", 0);
}

int main (int argc, char** argv)
{
    _name = "node_object_descriptor";
    //ros::init(argc, argv, _name, ros::init_options::NoSigintHandler); // Initialize ROS coms
    ros::init(argc, argv, _name); // Initialize ROS coms

    ros::NodeHandle n = ros::NodeHandle("~"); //The node handle
    //signal(SIGINT, mySigIntHandler);

    //// Override XMLRPC shutdown
    //ros::XMLRPCManager::instance()->unbind("shutdown");
    //ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    ObjectDescriptorDeepLearning< pcl::PointXYZRGBA > object_descriptor_deep_learning(&n); //initialize the class

    //Start program
    ros::Rate loop_rate(20);
    //while (ros::ok())
	//
	while (ros::ok())
	{
		ros::spinOnce();                   // Handle ROS events
		loop_rate.sleep();
	}

    //while (!g_request_shutdown)
    //{
        //loop_rate.sleep();
        //ros::spinOnce();                   // Handle ROS events
    //}

    //ROS_INFO("%s: Shutdown ros stuff ...", _name.c_str());
    //ros::shutdown();
    //ROS_INFO("%s: Shutdown node ...", _name.c_str());
    return 1;
}

