/**
 * @file
 * @brief 
 */
#ifndef _WRITE_OBJECT_PCD_CPP_
#define _WRITE_OBJECT_PCD_CPP_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//ros includes
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>

//pstreams includes
#include <pstreams/pstream.h>

//race ua includes
#include <race_perception_utils/print.h>
#include <race_perception_msgs/PCTOV.h>

/* _________________________________
   |                                 |
   |           Namespaces            |
   |_________________________________| */

using namespace ros;
using namespace std;
using namespace pcl;
using namespace race_perception_utils;

  //template <typename PointT> void 
  //fromROSMsg (const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>& cloud,
              //const MsgFieldMap& field_map)
  //{
    //// Copy info fields
    //cloud.header   = msg.header;
    //cloud.width    = msg.width;
    //cloud.height   = msg.height;
    //cloud.is_dense = msg.is_dense == 1;

    //// Copy point data
    //uint32_t num_points = msg.width * msg.height;
    //cloud.points.resize (num_points);
    //uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud.points[0]);

    //// Check if we can copy adjacent points in a single memcpy
    //if (field_map.size() == 1 &&
        //field_map[0].serialized_offset == 0 &&
        //field_map[0].struct_offset == 0 &&
        //msg.point_step == sizeof(PointT))
    //{
      //uint32_t cloud_row_step = static_cast<uint32_t> (sizeof (PointT) * cloud.width);
      //const uint8_t* msg_data = &msg.data[0];
      //// Should usually be able to copy all rows at once
      //if (msg.row_step == cloud_row_step)
      //{
        //memcpy (cloud_data, msg_data, msg.data.size ());
      //}
      //else
      //{
        //for (uint32_t i = 0; i < msg.height; ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
          //memcpy (cloud_data, msg_data, cloud_row_step);
      //}

    //}
    //else
    //{
      //// If not, memcpy each group of contiguous fields separately
      //for (uint32_t row = 0; row < msg.height; ++row)
      //{
        //const uint8_t* row_data = &msg.data[row * msg.row_step];
        //for (uint32_t col = 0; col < msg.width; ++col)
        //{
          //const uint8_t* msg_data = row_data + col * msg.point_step;
          //BOOST_FOREACH (const detail::FieldMapping& mapping, field_map)
          //{
            //memcpy (cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
          //}
          //cloud_data += sizeof (PointT);
        //}
      //}
    //}
  //}

  /** \brief Convert a PointCloud2 binary data blob into a pcl::PointCloud<T> object.
    * \param[in] msg the PointCloud2 binary blob
    * \param[out] cloud the resultant pcl::PointCloud<T>
    */
  //template<typename PointT> void 
  //fromROSMsg (const sensor_msgs::PointCloud2& msg, pcl::PointCloud<PointT>& cloud)
  //{
    //MsgFieldMap field_map;
    //createMapping<PointT> (msg.fields, field_map);
    //fromROSMsg (msg, cloud, field_map);
  //}

  /** \brief Convert a pcl::PointCloud<T> object to a PointCloud2 binary data blob.
    * \param[in] cloud the input pcl::PointCloud<T>
    * \param[out] msg the resultant PointCloud2 binary blob
    */
  //template<typename PointT> void 
  //toROSMsg (const pcl::PointCloud<PointT>& cloud, sensor_msgs::PointCloud2& msg)
  //{
    //// Ease the user's burden on specifying width/height for unorganized datasets
    //if (cloud.width == 0 && cloud.height == 0)
    //{
      //msg.width  = static_cast<uint32_t>(cloud.points.size ());
      //msg.height = 1;
    //}
    //else
    //{
      //assert (cloud.points.size () == cloud.width * cloud.height);
      //msg.height = cloud.height;
      //msg.width  = cloud.width;
    //}

    //// Fill point cloud binary data (padding and all)
    //size_t data_size = sizeof (PointT) * cloud.points.size ();
    //msg.data.resize (data_size);
    //memcpy (&msg.data[0], &cloud.points[0], data_size);

    //// Fill fields metadata
    //msg.fields.clear ();
    //for_each_type<typename traits::fieldList<PointT>::type> (detail::FieldAdder<PointT>(msg.fields));

    //msg.header     = cloud.header;
    //msg.point_step = sizeof (PointT);
    //msg.row_step   = static_cast<uint32_t> (sizeof (PointT) * msg.width);
    //msg.is_dense   = cloud.is_dense;
    ///// @todo msg.is_bigendian = ?;
  //}


//Global vars
string _name;
std::string pcin_topic;
std::string _obj_name;
int _pipeline_number;

template <class T>
std::string time_to_str(T ros_t)
{
	char buf[1024]      = "";
	time_t t = ros_t.sec;
	struct tm *tms = localtime(&t);
	strftime(buf, 1024, "%Y-%m-%d-%H-%M-%S", tms);
	return std::string(buf);
}


void pointCloudCallback(const race_perception_msgs::PCTOV::ConstPtr& msg)
{
	PrettyPrint pp;
	std::string date = time_to_str<ros::WallTime>(ros::WallTime::now());
	string file_path = ros::package::getPath("race_perception_utils") + "/pcd/" + _obj_name + "_" + date + ".pcd";

	PointCloud<PointXYZRGB>::Ptr pc;
	pc = (PointCloud<PointXYZRGB>::Ptr) new PointCloud<PointXYZRGB>;

	//PCLPointCloud2 aa;
	//pcl::fromROSMsg<PointXYZRGB>( aa, *pc);
	pcl::fromROSMsg<PointXYZRGB>( msg->point_cloud, *pc);
	//pcl::fromROSMsg( msg->point_cloud, *pc);
	//pcl::fromPCLPointCloud2<PointXYZRGB>( msg->point_cloud, *pc);

	PCDWriter writer; 
    writer.write<PointXYZRGB>(file_path, *pc, false/*non binary, i.e., ascii*/);

    pcl::io::savePCDFileASCII (file_path, *pc);
	//PCDWriter writer; 
	//writer.write(file_path, *pc, false[>non binary, i.e., ascii<]);

	//print info
	pp.info(std::ostringstream().flush() << "Saved point cloud from topic " << pcin_topic << " to " << file_path);
	pp.printCallback();

	ros::shutdown();
}


int main (int argc, char** argv)
{
	PrettyPrint pp;

	ros::init(argc, argv, "writeobjectpcd"); // Initialize ROS coms

	ros::NodeHandle* n = (ros::NodeHandle*) new ros::NodeHandle("~"); //The node handle

	//get node name
	_name = n->getNamespace();

	n->param<int>("pnum", _pipeline_number, 0);
	n->param<string>("obj_name", _obj_name, "unknown");

	//configure the subscriber
	pcin_topic = "/perception/pipeline" + boost::lexical_cast<string>(_pipeline_number) + "/tracker/tracked_object_point_cloud";
	ros::Subscriber sub = n->subscribe (pcin_topic, 1, &pointCloudCallback);

	//Start program
	pp.info(std::ostringstream().flush() << "Waiting for point cloud on topic " << pcin_topic);
	pp.printInitialization();

	ros::Rate loop_rate(20);
	while (ros::ok())
	{
		loop_rate.sleep(); //sleep
		ros::spinOnce(); // Handle ROS events

		PrettyPrint pp;

	}

	return 1;
}

#endif
