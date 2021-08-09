#ifndef _PERCEPTION_DB_NODELET_H_
#define _PERCEPTION_DBNODELET__H_

//ROS includes
#include <nodelet/nodelet.h>
//#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

//#include <boost/lexical_cast.hpp>

//#include <assert.h>
//#include <iostream>
#include <leveldb/db.h>
#include <leveldb/slice.h>

#include <race_perception_db/perception_db.h>
#include <race_perception_utils/print.h>

#include <race_perception_db/Slice.h>
#include <race_perception_db/get_db_address.h>
#include <race_perception_db/Put.h>
#include <race_perception_db/Delete.h>
#include <race_perception_db/Get.h>
#include <race_perception_db/LastNodeID.h>
#include <race_perception_db/GetSITOVs.h>
#include <race_perception_db/GetAllObjectCat.h>
#include <race_perception_db/GetKeys.h>
#include <race_perception_db/GetRTOV.h>
#include <race_perception_db/GetRTOVs.h>
#include <race_perception_db/GetPDBReport.h>


/* _________________________________
   |                                 |
   |        Class definition         |
   |_________________________________| */


namespace race_perception_db
{
    //enum PDBService
    //{
    //    Get_db_address,
    //    Put,
    //    Delete,
    //    Get,
    //    LastNodeID,
    //    GetSITOVs,
    //    GetAllObjectCat,
    //    GetRTOVs,
    //    GetPDBReport 
    //}
    //
    //const char* services[] = {
    //    "/get_db_address",
    //    "/perception/Put",
    //    "/Delete",
    //    "/Get",
    //    "/LastNodeID",
    //    "/GetSITOVs",
    //    "/GetAllObjectCat",
    //    "/GetRTOVs",
    //    "/GetPDBReport" 
    //}


	class PerceptionDBNodelet: public PerceptionDB, public nodelet::Nodelet
    {
		private:

			/* _________________________________
			   |                                 |
			   |           VARIABLES             |
			   |_________________________________| */

			//Type defs

			//local variables
			boost::shared_ptr<ros::Subscriber> _p_pcin_subscriber;
            boost::shared_ptr<ros::ServiceServer> _p_address_service;
            boost::shared_ptr<ros::ServiceServer> _p_put_service;
            boost::shared_ptr<ros::ServiceServer> _p_del_service;
            boost::shared_ptr<ros::ServiceServer> _p_get_service;
            boost::shared_ptr<ros::ServiceServer> _p_lastnid_service;
            boost::shared_ptr<ros::ServiceServer> _p_get_sitovs_service;
            boost::shared_ptr<ros::ServiceServer> _p_get_allobjectcat_service;
            boost::shared_ptr<ros::ServiceServer> _p_get_keys_service;
            boost::shared_ptr<ros::ServiceServer> _p_get_rtov_service;
            boost::shared_ptr<ros::ServiceServer> _p_get_rtovs_service;
            boost::shared_ptr<ros::ServiceServer> _p_pdbreport_service;


			leveldb::DB* _db;
			leveldb::Options _options;


			/* _________________________________
			   |                                 |
			   |           PARAMETERS  			|
			   |_________________________________| */

			std::string _db_path;
            std::string _pdb_source;

			bool _scratch;
			bool _w_mode;
			/* _________________________________
			   |                                 |
			   |     PRIVATE CLASS METHODS       |
			   |_________________________________| */


	        std::string convertPointerToStringAddress(const leveldb::DB* p_db);
	        void convertAddressStringToPointer(const std::string& address, leveldb::DB** p_db);

            bool connectDB();

	        bool cb_get_db_address(race_perception_db::get_db_address::Request  &req, race_perception_db::get_db_address::Response &res);
	        bool cb_Put(race_perception_db::Put::Request &req, race_perception_db::Put::Response &res);
	        bool cb_Delete(race_perception_db::Delete::Request &req, race_perception_db::Delete::Response &res);
	        bool cb_Get(race_perception_db::Get::Request &req, race_perception_db::Get::Response &res);
	        bool cb_LastNodeID(race_perception_db::LastNodeID::Request &req, race_perception_db::LastNodeID::Response &res);
	        bool cb_GetSITOVs(race_perception_db::GetSITOVs::Request &req, race_perception_db::GetSITOVs::Response &res);
	        bool cb_GetAllObjectCat(race_perception_db::GetAllObjectCat::Request  &req, race_perception_db::GetAllObjectCat::Response &res);
	        bool cb_GetKeys(race_perception_db::GetKeys::Request &req, race_perception_db::GetKeys::Response &res);
	        bool cb_GetRTOVs(race_perception_db::GetRTOVs::Request &req, race_perception_db::GetRTOVs::Response &res);
	        bool cb_GetRTOV(race_perception_db::GetRTOV::Request &req, race_perception_db::GetRTOV::Response &res);
	        bool cb_GetPDBReport(race_perception_db::GetPDBReport::Request &req, race_perception_db::GetPDBReport::Response &res);

            std::vector <race_perception_msgs::SITOV> querySITOVs(const std::string rtov_key);
            std::vector <race_perception_msgs::ObjectCategory> queryAllObjectCat();
            std::vector <std::string> queryKeys(const std::string k_name);
            race_perception_msgs::RTOV queryRTOV(const std::string key);
            std::vector <std::string> queryRTOVs(const unsigned int tid);
            //std::vector <std::string> queryPDBReport();
            race_perception_utils::MeasurePDB queryPDBReport();

		public:
			PerceptionDBNodelet(); //empty constructor  
			PerceptionDBNodelet(ros::NodeHandle* n, bool flg_is_nodelet = false, bool flg_is_pdb = false);

			virtual ~PerceptionDBNodelet(); //destructor  


			/* _________________________________
			   |                                 |
			   |           CLASS METHODS         |
			   |_________________________________| */


			virtual void onInit(); //mandatory method for nodelet manager to call

            virtual bool put(const leveldb::Slice& key, const leveldb::Slice& value);
            virtual bool del(const leveldb::Slice& key);
            virtual bool get(const leveldb::Slice& key, std::string* value);

            virtual unsigned int getLastNodeID(const std::string name, const std::string id_name);
            virtual unsigned int getLastTrkID();
            virtual std::string makeKey(const std::string k_name, const unsigned int tid, const unsigned int vid);
            virtual std::string makeKey(const std::string k_name, const unsigned int level, const unsigned int tid, const unsigned int vid);
            virtual std::string makeSIKey(const std::string k_name, const unsigned int tid, const unsigned int vid, const unsigned int sid);
            virtual std::string makeOCKey(const std::string k_name, const std::string cat_name, const unsigned int cid);
            virtual std::string makeTopicName(const std::string t_name, const unsigned int nid);

            virtual std::vector <race_perception_msgs::SITOV> getSITOVs(const std::string rtov_key);

            virtual std::vector <race_perception_msgs::ObjectCategory> getAllObjectCat();
            virtual std::vector <std::string> getKeys(const std::string k_name);

            virtual race_perception_msgs::RTOV getRTOV(const std::string key);
            virtual std::vector <std::string> getRTOVs(const unsigned int tid);
            virtual race_perception_utils::MeasurePDB reportPDB();
            //virtual std::vector <std::string> reportPDB();

			/* _________________________________
			   |                                 |
			   |           ACCESSORS             |
			   |_________________________________| */


    };

}

#endif
