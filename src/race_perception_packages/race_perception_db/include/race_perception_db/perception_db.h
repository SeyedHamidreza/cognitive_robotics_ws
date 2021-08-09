#ifndef _PERCEPTION_DB_H_
#define _PERCEPTION_DB_H_

//ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

//#include <boost/lexical_cast.hpp>

//#include <assert.h>
//#include <iostream>
#include <vector>

#include <leveldb/db.h>
#include <leveldb/slice.h>

#include <race_perception_msgs/perception_msgs.h>
#include <race_perception_utils/MeasurePDB.h>

/* _________________________________
   |                                 |
   |        Class definition         |
   |_________________________________| */


namespace race_perception_db
{
	class PerceptionDB
	{
			
		protected:

			/* _________________________________
			   |                                 |
			   |           VARIABLES             |
			   |_________________________________| */

			//Type defs

			//local variables
			std::string _name;
			bool _verb;
			ros::NodeHandle* _p_nh; // The pointer to the node handle
			ros::NodeHandle _nh; // The node handle
			ros::NodeHandle _priv_nh; // The node handle
			ros::NodeHandle* _p_priv_nh; // The node handle
			bool _flg_is_nodelet; //a flag to check if this code is running as a node or a nodelet

			bool _flg_is_pdb; //a flag to check if this code is running as a Perception DB
			bool _flg_same_proc; //a flag to check if it runs in the same process with PDB. It should set true only in default construct.

			/* _________________________________
			   |                                 |
			   |           CONSTRUCTORS          |
			   |_________________________________| */

		protected:
			PerceptionDB(); //empty constructor  
            void operator=(const PerceptionDB&);

			PerceptionDB(ros::NodeHandle* n, bool flg_is_nodelet = false, bool flg_is_pdb = false);

		public:


			static PerceptionDB* getPerceptionDB(ros::NodeHandle* n, bool flg_is_nodelet = false, bool flg_is_pdb = false);

			/**
			 * @brief Destructor
			 */
			virtual ~PerceptionDB();

			/* _________________________________
			   |                                 |
			   |           CLASS METHODS         |
			   |_________________________________| */


			//virtual void onInit() = 0; //mandatory method for nodelet manager to call

            virtual bool put(const leveldb::Slice& key, const leveldb::Slice& value) = 0;
            virtual bool del(const leveldb::Slice& key) = 0;
            virtual bool get(const leveldb::Slice& key, std::string* value) = 0;

            virtual unsigned int getLastNodeID(const std::string name, const std::string id_name = race_perception_msgs::SEP) = 0;
            virtual unsigned int getLastTrkID() = 0;
            virtual std::string makeKey(const std::string k_name, const unsigned int tid, const unsigned int vid) = 0;
            virtual std::string makeKey(const std::string k_name, const unsigned int level, const unsigned int tid, const unsigned int vid) = 0;
            virtual std::string makeSIKey(const std::string k_name, const unsigned int tid, const unsigned int vid, const unsigned int sid) = 0;
            virtual std::string makeOCKey(const std::string k_name, const std::string cat_name, const unsigned int cid) =0;
            virtual std::string makeTopicName(const std::string t_name, const unsigned int nid) = 0;

            virtual std::vector <race_perception_msgs::SITOV> getSITOVs(const std::string rtov_key) = 0;
            virtual std::vector <race_perception_msgs::ObjectCategory> getAllObjectCat() = 0;

            virtual std::vector <std::string> getKeys(const std::string k_name) = 0;


            virtual race_perception_msgs::RTOV getRTOV(const std::string key) = 0;
            virtual std::vector <std::string> getRTOVs(const unsigned int tid) = 0;
            //virtual std::vector <std::string> reportPDB() = 0;
            virtual race_perception_utils::MeasurePDB reportPDB() = 0;

			/* _________________________________
			   |                                 |
			   |           ACCESSORS             |
			   |_________________________________| */


	};

}

#endif
