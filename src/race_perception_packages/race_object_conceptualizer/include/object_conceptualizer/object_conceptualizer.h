#ifndef _OBJECT_CONCEPTUALIZER_H_
#define _OBJECT_CONCEPTUALIZER_H_

/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>   
#include <tf/transform_listener.h>

//Gi includes
#include <pluginlib/class_list_macros.h> 
#include <stdio.h>
#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>
#include <race_perception_db/MsgTest.h>

//package includes
#include <feature_extraction/spin_image.h>
#include <race_perception_msgs/perception_msgs.h>
#include <object_conceptualizer/object_conceptualization.h>


//Eigen includes
#include <Eigen/Core>

//PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/features/spin_image.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/uniform_sampling.h>

//raceua includes
#include <race_object_conceptualizer/UserObjectLabel.h>
#include <race_perception_msgs/ObjectCategory.h>
#include <object_conceptualizer/object_conceptualization.h>

#include <race_perception_utils/cycle.h>
#include <race_perception_utils/print.h>
#include <visualization_msgs/MarkerArray.h>

/* _________________________________
   |                                 |
   |        Class definition         |
   |_________________________________| */

using namespace tf;
using namespace std;
using namespace pcl;
using namespace ros;
using namespace race_perception_msgs;
using namespace race_perception_db;
// 	using namespace tf;
using namespace race_perception_utils;
int tmp_view_id = 10000;

namespace race_object_conceptualizer
{
	template <class PointT>
		class ObjectConceptualizer: public nodelet::Nodelet 
	{
		public:

			//local variables
			unsigned int _cat_id_count;
			string _name;
			bool _verb;
			ros::NodeHandle* _p_nh; // The pointer to the node handle
			ros::NodeHandle _nh; // The node handle
			ros::NodeHandle _priv_nh; // The node handle
			ros::NodeHandle* _p_priv_nh; // The node handle
			bool _flg_is_nodelet; //a flag to check if this code is running as a node or a nodelet

			Publisher conceptualizer_marker_publisher;
			boost::shared_ptr<CycleDebug> cd;

			boost::shared_ptr<ros::Subscriber> _p_pcin_subscriber;
			boost::shared_ptr<ros::Subscriber> _p_pcin_subscriber2;

			boost::shared_ptr<ros::Publisher> _p_pcin_publisher;

			boost::shared_ptr<ros::Subscriber> _p_user_object_label_subscriber;
			PerceptionDB* _pdb; //initialize the class_list_macros

			/* _________________________________
			   |                                 |
			   |           PARAMETERS			|
			   |_________________________________| */

			// 			double _param1;
			// 			string _param2;

			/* _________________________________
			   |                                 |
			   |           CONSTRUCTORS          |
			   |_________________________________| */

			ObjectConceptualizer(){_flg_is_nodelet=true;};

			ObjectConceptualizer(ros::NodeHandle* n)
			{
				_flg_is_nodelet=false; 
				_p_nh = n; //if this is a node set both the nodehandle and private node handle to n
				_p_priv_nh = n; 
				onInit();
			};

			/**
			 * @brief Destructor
			 */
			~ObjectConceptualizer()
			{
				PrettyPrint pp(_name);
				pp.info(std::ostringstream().flush() << _name.c_str() << ": Destructor called");
				pp.info(std::ostringstream().flush() << _name.c_str() << ": Finished destructor");
				pp.printCallback();
			};


			/* _________________________________
			   |                                 |
			   |           CLASS METHODS         |
			   |_________________________________| */

			void onInit(void)
			{

				//create a node handle in internal nh_ variable, and point p_nh_
				//to it. Only done if we are using a nodelet.
				if (_flg_is_nodelet==true)
				{
					_nh = getNodeHandle(); 
					_p_nh = &_nh;
					_priv_nh = getPrivateNodeHandle(); 
					_p_priv_nh = &_priv_nh;
				}

				//Initialize tf stuff

				//initialize the category id 
				_cat_id_count=0; //TODO because we want persistence this will have to be given after reading the db

				//initialize parameters
				_name = _p_priv_nh->getNamespace();
				PrettyPrint pp(_name);

				//initialize the database
				_pdb = race_perception_db::PerceptionDB::getPerceptionDB(_p_priv_nh, _flg_is_nodelet); //initialize the database class

				//initialize the subscriber to a new concept message
				_p_user_object_label_subscriber = (boost::shared_ptr<ros::Subscriber>) new ros::Subscriber;
				*_p_user_object_label_subscriber = _p_priv_nh->subscribe ("user_object_label", 100, &ObjectConceptualizer::callbackUserObjectLabel, this);

				cd = (boost::shared_ptr<CycleDebug>) new CycleDebug(_p_nh, _name);

				conceptualizer_marker_publisher = _p_nh->advertise<visualization_msgs::MarkerArray>("/perception/conceptualizer/neat_markers", 100);


				//Output initialization information
				if (_verb) 
					pp.info(std::ostringstream().flush() << "OnInit race_object_conceptualizer finished ");

				pp.printInitialization();

			};


			/**
			 * @brief Callback for creating a new concept
			 * Can emulate a publication of this message in the shell by doing:
			 *
			 *  rostopic pub /object_conceptualizer/user_object_label
			 *  race_object_conceptualizer/UserObjectLabel "header:
			 *  seq: 0
			 *  stamp:
			 *  secs: 0
			 *  nsecs: 0
			 *  frame_id: ''
			 *  track_id: 0
			 *  view_id: 0
			 *  category_name: 'Mug'"
			 *
			 * @param msg_in
			 */
			void callbackUserObjectLabel(const race_object_conceptualizer::UserObjectLabel &msg_in)
			{
				/* _________________________________
				   |                                 |
				   | Read data from given category   |
				   |_________________________________| */

				ros::Time beginProc = ros::Time::now();

				PrettyPrint pp(_name);
				pp.info(std::ostringstream().flush() << "conceptualizing "<<msg_in.category_name.c_str() << " category based on track_id=" << msg_in.track_id << " ,view_id="<< msg_in.view_id);

				//_cat_id_count++; //increment the cat_id_count

				//create an object_view_key 
				//std::string object_view_key = _pdb->makeKey(key::RV, track_id, view_id);

				//Prepare an ObjectCategory msg for writing a key to the db
				race_perception_msgs::ObjectCategory oc;
				oc.header.frame_id = "";
				oc.cat_name = msg_in.category_name;
				oc.cat_id = 0; 

				//Check for the value of view id
				if (msg_in.view_id != -1)
				{
					pp.error(std::ostringstream().flush() << "View id != -1. This behaviour is not implemented. I will use as if its equal to -1");
				}


				//Check if the cat exists (we should update) or if we have to create a new cat
				std::string oc_key = _pdb->makeOCKey(key::OC, oc.cat_name, oc.cat_id); //create an OC key	
				std::string str_oc;
				_pdb->get(oc_key, &str_oc);
				uint32_t oc_size = str_oc.length();

				if (oc_size != 0) //Category exists in the DB already, then deserialize the DB buffer into OC
				{
					boost::shared_array<uint8_t> oc_dbuffer(new uint8_t[oc_size]);
					memcpy(oc_dbuffer.get(), str_oc.data(), str_oc.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

					//deserialize Msg 
					race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, race_perception_msgs::ObjectCategory>::deserialize(oc_dbuffer, oc, oc_size);
					pp.info(std::ostringstream().flush() << "Object Category already contains " <<  oc.rtov_keys.size() << " views in the DB");
				}
				//else -> nothing to do because we will use an empty oc object declared above

				//update the time stamp
				oc.header.stamp = ros::Time::now();

				//Get all rtovs in the DB for this track id
				std::vector<std::string> rtovs_in_db = _pdb->getRTOVs(msg_in.track_id);
				pp.info(std::ostringstream().flush() << "Track id " <<  msg_in.track_id << " has " << rtovs_in_db.size() << " views in the DB");

				//Add the new rtov keys to the rtov_keys already in oc (add new views to object cat)
				for (size_t i=0; i < rtovs_in_db.size(); i++)
				{
					bool add_key = true;
					for (size_t j=0; j < oc.rtov_keys.size(); j++)
					{
						if (oc.rtov_keys.at(j) == rtovs_in_db.at(i))
						{
							add_key = false;
						}

					}

					if (add_key)
					{
						oc.rtov_keys.push_back(rtovs_in_db.at(i));  
					}
				}

				pp.info(std::ostringstream().flush() << "After updating oc has " <<  oc.rtov_keys.size() << " views");


				//Creat a rtov key for pointed object
				//std::string _rtov_key = _pdb->makeKey(key::RV, msg_in.track_id, msg_in.view_id);
				//std::string _rtov_key = _pdb->makeKey(key::RV, msg_in.track_id, 1);
				//std::string _rtov_key = _pdb->makeKey(key::RV, msg_in.track_id, msg_in.view_id);
				//Push back to OC
				//oc.rtov_keys.push_back(_rtov_key);

				//Timer toc
				ros::Duration duration = ros::Time::now() - beginProc;
				double duration_sec = duration.toSec();
				pp.info(std::ostringstream().flush() << "Read data from given category took " << duration_sec << " secs");


				/* _______________________________
				|                                |
				|     Subsampling object views   |
				|________________________________| */
				beginProc = ros::Time::now();
				// // 				strategy 1: Just use the Key view, only the key views could add to the db
				// // 				in reading part , the checked the object tracked is keyview or no.
				// 				vector < string > subsampleRTOVKeys;
				// std::vector <std::vector<SITOV> > category_instances;
				// 				for (size_t i = 0; i < oc.rtov_keys.size(); i++)
				// 				{
				// 				    
				// 				    RTOV rtov; 
				// 				    string value;
				// 				    std::string _rtov_key = _pdb->makeKey(key::RV, msg_in.track_id, msg_in.view_id);
				//  				    std::string _rtov_key = oc.rtov_keys.at(i);
				// 				    _pdb->get(_rtov_key, &value);
				// 				    uint32_t deserial_size = value.length(); 
				// 
				// 				    Declare a shared array 
				// 				    boost::shared_array<uint8_t> buffer(new uint8_t[deserial_size]);
				// 				    memcpy(buffer.get(), value.data(), value.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!
				// 
				// 				    deserialize Msg 
				// 				    race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::deserialize(buffer, rtov, deserial_size);	
				// 				
				// 				    pp.info(std::ostringstream().flush() << "rtov.is_key_view = " << category_instances.size());
				// 
				// 				    if (rtov.is_key_view)
				// 				    {
				// 					pp.info(std::ostringstream().flush() << oc.cat_name.c_str() <<".RTOV_key("<<i<<") = " << oc.rtov_keys.at(i).c_str());
				// 					subsampleRTOVKeys.push_back(oc.rtov_keys.at(i).c_str());
				// 					std::vector<SITOV> object_view_spin_images = _pdb->getSITOVs(oc.rtov_keys.at(i).c_str());
				// 					category_instances.push_back(object_view_spin_images);
				// 				    }
				// 				}
				// 				
				// 				duration = (ros::Time::now() - beginProc);
				// 				duration_sec = duration.toSec();
				// 				pp.info(std::ostringstream().flush() << "category size (Just KeyViews) = " << subsampleRTOVKeys.size());
				// 				pp.info(std::ostringstream().flush() << "Checking all RTOVs Fields for given category took " << duration_sec << " secs");
				// 
				// 				
				// 				clear previous data 
				//     				oc.rtov_keys.clear();
				//     				assign subsample rtov keys to given category
				//     				
				//     				for (size_t i=0; i<subsampleRTOVKeys.size(); i++)
				//     				{
				//     					oc.rtov_keys.push_back(subsampleRTOVKeys.at(i));
				//     				}
				//     			


				//strategy 2: select fix=10 object view 
				//std::vector <std::vector<SITOV> > category_instances;
				//int stepSize=1;
				//int numberOfObjectViewPerGategory = 20;
				//if (oc.rtov_keys.size() > numberOfObjectViewPerGategory)
				//{
				//stepSize = int (oc.rtov_keys.size()/numberOfObjectViewPerGategory);
				//}

				//pp.info(std::ostringstream().flush() << "StepSize= " <<stepSize );
				//pp.info(std::ostringstream().flush() << "category size " << oc.rtov_keys.size());
				//// 				

				//vector < string > subsampleRTOVKeys ;
				//for (int i = 0; i < oc.rtov_keys.size(); i+=stepSize)
				//{
				//pp.info(std::ostringstream().flush() << oc.cat_name.c_str() <<".RTOV_key("<<i<<") (subsampled)= " << oc.rtov_keys.at(i).c_str());
				//subsampleRTOVKeys.push_back(oc.rtov_keys.at(i).c_str());
				//}

				////clear previous data 
				//oc.rtov_keys.clear();
				//// assign subsample rtov keys to given category

				//for (size_t i=0; i<subsampleRTOVKeys.size(); i++)
				//{
				//oc.rtov_keys.push_back(subsampleRTOVKeys.at(i));
				//}

				std::vector <std::vector<SITOV> > category_instances;
				for (size_t i = 0; i < oc.rtov_keys.size(); i++)
				{
					pp.info(std::ostringstream().flush() << oc.cat_name.c_str() <<".RTOV_key("<<i<<") = " << oc.rtov_keys.at(i).c_str());
					std::vector<SITOV> object_view_spin_images = _pdb->getSITOVs(oc.rtov_keys.at(i).c_str());
					category_instances.push_back(object_view_spin_images);
				}

				pp.info(std::ostringstream().flush() << "Category size for computing ICD = " << category_instances.size());
				//pp.info(std::ostringstream().flush() << "Category size after subsampling = " << oc.rtov_keys.size());

				//Timer toc
				duration = (ros::Time::now() - beginProc);
				duration_sec = duration.toSec();
				pp.info(std::ostringstream().flush() << "Subsampling took " << duration_sec << " secs");

				if (category_instances.size() > 0)
				{

					/* ________________________________
					|                                  |
					|  Comput ICD for given category   |
					|__________________________________| */
					beginProc = ros::Time::now();


					//float new_ICD = 0;
					intraCategoryDistance(category_instances, oc.icd, pp);	
					//oc.icd = new_ICD;

					//pp.info(std::ostringstream().flush() << oc.cat_name.c_str() << " category has " << oc.rtov_keys.size() << " objects.");
					pp.info(std::ostringstream().flush() << "ICD for " << oc.cat_name.c_str() << " updated. New ICD = "<< oc.icd );
					//pp.printCallback();

					// 				endProc =ros::Time::now().toSec();
					// 				duration = endProc-beginProc;

					//Timer toc
					duration = (ros::Time::now() - beginProc);
					duration_sec = duration.toSec();
					pp.info(std::ostringstream().flush() << "ICD Computation took " << duration_sec << " secs");

					/* ___________________________________
					   |                                  |
					   |   Write data to given category   |
					   |__________________________________| */
					beginProc =ros::Time::now();

					oc_size = ros::serialization::serializationLength(oc);

					boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
					PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::serialize(oc_buffer, oc, oc_size);	
					leveldb::Slice ocs((char*)oc_buffer.get(), oc_size);
					_pdb->put(oc_key, ocs);

					//Timer toc
					duration = ros::Time::now() - beginProc;
					duration_sec = duration.toSec();
					pp.info(std::ostringstream().flush() << "write data to db took " << duration_sec << " secs");


					/* _________________________________
					   |                                  |
					   |   	visualization         |
					   |_________________________________| */
					beginProc =ros::Time::now();
					set_neat_visualization_marker_array( msg_in.track_id, msg_in.category_name, true);
					duration = ros::Time::now() - beginProc;
					duration_sec = duration.toSec();
					pp.info(std::ostringstream().flush() << "visualization took " << duration_sec << " secs");
				}
				else
				{
					beginProc =ros::Time::now();
					set_neat_visualization_marker_array( msg_in.track_id, msg_in.category_name, false);
					duration = ros::Time::now() - beginProc;
					duration_sec = duration.toSec();
					pp.info(std::ostringstream().flush() << "visualization took " << duration_sec << " secs");
				}   

				vector <ObjectCategory> ListOfObjectCategory = _pdb->getAllObjectCat();
				ROS_INFO("number of object categories in database = %i", ListOfObjectCategory.size());

				updateNaiveBayesModel1(msg_in.category_name.c_str(), 0, pp, _pdb );

				pp.printCallback();				
			}

			int set_neat_visualization_marker_array(int track_id, std::string category_name , bool flag)
			{

				//STEP 1: need to get the position of the object so we can draw
				//text nearby
				tf::StampedTransform stf; //the transform
				std::string tracker_frame_id = "/perception/pipeline" + boost::lexical_cast<std::string>(track_id) + "/tracker";

				//now we can publish the outcome of object_recognition as a text marker
				visualization_msgs::MarkerArray marker_array; 
				geometry_msgs::Point p;
				//double pt_size = 0.004;
				double duration = 2;

				/* _________________________________
				   |                                 |
				   |         DRAW TEXT INFO          |
				   |_________________________________| */
				if (1)
				{
					visualization_msgs::Marker marker;
					marker.header.frame_id = tracker_frame_id;
					marker.header.stamp = ros::Time::now();
					marker.ns = "information";
					marker.id = track_id;
					marker.frame_locked = true;
					marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					marker.action = visualization_msgs::Marker::ADD;

					//marker.pose.position.x = 0.2;
					marker.pose.position.z = 0.01;

					marker.scale.z = 0.03; 
					marker.color.r  = 1; marker.color.g  = 0; marker.color.b  = 0; marker.color.a = 1;
					marker.lifetime = Duration(duration);

					std::string result = "";
					if (flag)
					{
						result = "conceptualizing " ;
						result+= category_name.c_str();
						result+= " category";
					}
					else
					{
						result = "There is not enough key views for conceptualizing";
					}
					marker.text = result;
					marker_array.markers.push_back(marker);
				}


				conceptualizer_marker_publisher.publish(marker_array);
				return 1;

			}

			/* _________________________________
			   |                                 |
			   |           ACCESSORS             |
			   |_________________________________| */


	};

	class ObjectConceptualizer_Nodelet: public ObjectConceptualizer<pcl::PointXYZRGBA>{};


	//PLUGINLIB_DECLARE_CLASS(<nodelet namespace>, <nodelet class>, <nodelet namespace>::<nodelet class>, nodelet::Nodelet);
	//PLUGINLIB_DECLARE_CLASS(race_object_conceptualizer, ObjectConceptualizer_Nodelet, race_object_conceptualizer::ObjectConceptualizer_Nodelet, nodelet::Nodelet);
	PLUGINLIB_EXPORT_CLASS( race_object_conceptualizer::ObjectConceptualizer_Nodelet, nodelet::Nodelet);

}//end feature_extraction namespace
#endif
