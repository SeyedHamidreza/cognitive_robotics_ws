#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>

#include <stdlib.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include <assert.h>
#include <iostream>
#include <leveldb/db.h>

#include <race_perception_msgs/perception_msgs.h>
#include <race_perception_db/perception_db_serializer.h>

#include "perception_db_nodelet.h"


//PLUGINLIB_DECLARE_CLASS(race_perception_db, PerceptionDBNodelet,  race_perception_db::PerceptionDBNodelet, nodelet::Nodelet);

PLUGINLIB_EXPORT_CLASS(race_perception_db::PerceptionDBNodelet, nodelet::Nodelet);

using namespace std;
using namespace race_perception_msgs;
using namespace race_perception_utils;

namespace race_perception_db
{

    PerceptionDB::PerceptionDB()
    {
        //_flg_is_nodelet = true; 
        //_flg_is_pdb = true;

        //_flg_same_proc = true;

        //ROS_INFO("new PerceptionDB without argument!!");
    };

    PerceptionDB::PerceptionDB(ros::NodeHandle* n, bool flg_is_nodelet, bool flg_is_pdb)
    {
        _flg_is_nodelet = flg_is_nodelet; 
        _flg_is_pdb = flg_is_pdb;
        _flg_same_proc = _flg_is_nodelet || _flg_is_pdb; //TODO: 
        _p_nh = n; //if this is a node set both the nodehandle and private node handle to n
        _p_priv_nh = n;

        ROS_INFO("new PerceptionDB with arguments: flg_is_nodelet: %d, flg_is_pdb: %d: _flg_same_proc: %d", _flg_is_nodelet, _flg_is_pdb, _flg_same_proc);
    };

    PerceptionDB* PerceptionDB::getPerceptionDB(ros::NodeHandle* n, bool flg_is_nodelet, bool flg_is_pdb)
    {
        PerceptionDBNodelet* p_pdb = new PerceptionDBNodelet(n, flg_is_nodelet, flg_is_pdb);

        return p_pdb; 
    };

    PerceptionDB::~PerceptionDB()
    {
        if (_verb) ROS_INFO("%s: Destructor called", _name.c_str());
    }

    PerceptionDBNodelet::PerceptionDBNodelet()
    {
        _flg_is_nodelet = true; 
        _flg_is_pdb = true;
        _flg_same_proc = true;

        ROS_INFO("new PerceptionDBNodelet without argument!!");
    };

    PerceptionDBNodelet::PerceptionDBNodelet(ros::NodeHandle* n, bool flg_is_nodelet, bool flg_is_pdb)
        : PerceptionDB(n, flg_is_nodelet, flg_is_pdb)
    {
        // _flg_same_proc = false;

        onInit();

        ROS_INFO("new PerceptionDBNodelet with arguments in %s !!", _name.c_str());
    };

    PerceptionDBNodelet::~PerceptionDBNodelet()
    {
        if (_verb) ROS_INFO("%s: Destructor called", _name.c_str());
        if(_flg_is_pdb)
            delete _db; 
    }


			/* _________________________________
			   |                                |
			   |     callback functions  		|
			   |________________________________| */

    bool PerceptionDBNodelet::cb_Put(race_perception_db::Put::Request  &req, race_perception_db::Put::Response &res)
    {
        //Dont care about the request, just pass the db pointers address
        //to a string in response
        bool ret = false;

        leveldb::Slice key(reinterpret_cast<char*> (&req.key.data[0]), req.key.size);
        leveldb::Slice value(reinterpret_cast<char*> (&req.value.data[0]), req.value.size);

        ret = this->put(key, value);
        //leveldb::Status s = _db->Put(leveldb::WriteOptions(), key, value);
        res.status = ret;

        return ret;
    }

    bool PerceptionDBNodelet::cb_Delete(race_perception_db::Delete::Request  &req, race_perception_db::Delete::Response &res)
    {
        //Dont care about the request, just pass the db pointers address
        //to a string in response
        bool ret;

        leveldb::Slice key(reinterpret_cast<char*> (&req.key.data[0]), req.key.size);

        ret = this->del(key);
        res.status = ret;

        return ret;
    }

    bool PerceptionDBNodelet::cb_Get(race_perception_db::Get::Request  &req, race_perception_db::Get::Response &res)
    {
        //Dont care about the request, just pass the db pointers address
        //to a string in response
        bool ret;

        leveldb::Slice key(reinterpret_cast<char*> (&req.key.data[0]), req.key.size);
        string value;

        ret = this->get(key, &value);
        res.value = value;
        res.status = ret;

        return ret;
    }

    bool PerceptionDBNodelet::cb_LastNodeID(race_perception_db::LastNodeID::Request  &req, race_perception_db::LastNodeID::Response &res)
    {
        //Dont care about the request, just pass the db pointers address
        //to a string in response
        //if (_verb) ROS_INFO("%s: cb_LastNodeID called by name: %s, id_name:%s", _name.c_str(), req.name.c_str(), req.id_name.c_str());

        unsigned int ui = this->getLastNodeID(req.name, req.id_name);
        //if (_verb) ROS_INFO("%s: cb_LastNodeID result :%u", _name.c_str(), ui);
        res.nid = ui;

        return true;
    }

    bool PerceptionDBNodelet::cb_GetSITOVs(race_perception_db::GetSITOVs::Request  &req, race_perception_db::GetSITOVs::Response &res)
    {

        //if (_verb) ROS_INFO("%s: cb_GetSITOVs called!", _name.c_str());

        res.sitovs = this->querySITOVs(req.rtov_key);

        return true;
    }

    bool PerceptionDBNodelet::cb_GetAllObjectCat(race_perception_db::GetAllObjectCat::Request  &req, race_perception_db::GetAllObjectCat::Response &res)
    {

        //if (_verb) ROS_INFO("%s: cb_GetAllObjectCat called!", _name.c_str());

        res.ocs = this->queryAllObjectCat();

        return true;
    }

	bool PerceptionDBNodelet::cb_GetKeys(race_perception_db::GetKeys::Request &req, race_perception_db::GetKeys::Response &res)
    {

        //if (_verb) ROS_INFO("%s: cb_GetKeys called!", _name.c_str());

        res.keys = this->queryKeys(req.k_name);

        return true;
    }

    bool PerceptionDBNodelet::cb_GetRTOV(race_perception_db::GetRTOV::Request &req, race_perception_db::GetRTOV::Response &res)
    {

        //if (_verb) ROS_INFO("%s: cb_GetRTOVs called!", _name.c_str());

        res.rtov = this->queryRTOV(req.key);

        return true;
    }

    bool PerceptionDBNodelet::cb_GetRTOVs(race_perception_db::GetRTOVs::Request &req, race_perception_db::GetRTOVs::Response &res)
    {

        //if (_verb) ROS_INFO("%s: cb_GetRTOVs called!", _name.c_str());

        res.rtov_key = this->queryRTOVs(req.tid);

        return true;
    }

    bool PerceptionDBNodelet::cb_GetPDBReport(race_perception_db::GetPDBReport::Request  &req, race_perception_db::GetPDBReport::Response &res)
    {

        //if (_verb) ROS_INFO("%s: cb_GetPDBReport called!", _name.c_str());

        res.report = this->queryPDBReport();

        return true;
    }

    string PerceptionDBNodelet::convertPointerToStringAddress(const leveldb::DB* p_db)
    {
        intptr_t address(reinterpret_cast<intptr_t>(p_db));
        stringstream ss;
        ss << address;
        return ss.str();
    }

    void PerceptionDBNodelet::convertAddressStringToPointer(const string& address, leveldb::DB** p_db)
    {
        stringstream ss;
        ss << address;
        int tmp(0);
        if(!(ss >> tmp)) ROS_ERROR("Failed to convert address string to pointer");
        *p_db = reinterpret_cast<leveldb::DB*>(tmp);
    }

    bool PerceptionDBNodelet::cb_get_db_address(race_perception_db::get_db_address::Request  &req, race_perception_db::get_db_address::Response &res)
    {
        //Dont care about the request, just pass the db pointers address
        //to a string in response

        res.mem_address = convertPointerToStringAddress(_db);
        if (_verb) NODELET_INFO("%s: service call get_db_address as: %s ", _name.c_str(), res.mem_address.c_str());

        return true;
    }


    void PerceptionDBNodelet::onInit() //mandatory method for nodelet manager to call
    {
        PrettyPrint pp;

        pp.info(std::ostringstream().flush() << "onInit in PerceptionDBNodelet!!");
        //create a node handle in internal nh_ variable, and point p_nh_
        //to it. Only done if we are using a nodelet.
        if (_flg_is_pdb && _flg_is_nodelet)
        {
            _nh = getMTNodeHandle(); 
            _p_nh = &_nh;
            _priv_nh = getMTPrivateNodeHandle(); 
            _p_priv_nh = &_priv_nh;

            //_p_address_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;

            //*_p_address_service = _p_nh->advertiseService("/perception/pdb/get_db_address", &PerceptionDBNodelet::cb_get_db_address, this);
            //if (_verb) NODELET_INFO("Start service /perception/pdb/get_db_address");

        }

        if (_flg_is_pdb)
        {
            pp.info(std::ostringstream().flush() << "_flg_is_pdb: true");
            _p_put_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_del_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_get_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_lastnid_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_get_sitovs_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_get_allobjectcat_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_get_keys_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_get_rtov_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_get_rtovs_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;
            _p_pdbreport_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;


            *_p_put_service = _p_nh->advertiseService("/perception/pdb/Put", &PerceptionDBNodelet::cb_Put, this);
            *_p_del_service = _p_nh->advertiseService("/perception/pdb/Delete", &PerceptionDBNodelet::cb_Delete, this);
            *_p_get_service = _p_nh->advertiseService("/perception/pdb/Get", &PerceptionDBNodelet::cb_Get, this);
            *_p_lastnid_service = _p_nh->advertiseService("/perception/pdb/LastNodeID", &PerceptionDBNodelet::cb_LastNodeID, this);
            *_p_get_sitovs_service = _p_nh->advertiseService("/perception/pdb/GetSITOVs", &PerceptionDBNodelet::cb_GetSITOVs, this);
            *_p_get_allobjectcat_service = _p_nh->advertiseService("/perception/pdb/GetAllObjectCat", &PerceptionDBNodelet::cb_GetAllObjectCat, this);
            *_p_get_keys_service = _p_nh->advertiseService("/perception/pdb/GetKeys", &PerceptionDBNodelet::cb_GetKeys, this);
            *_p_get_rtov_service = _p_nh->advertiseService("/perception/pdb/GetRTOV", &PerceptionDBNodelet::cb_GetRTOV, this);
            *_p_get_rtovs_service = _p_nh->advertiseService("/perception/pdb/GetRTOVs", &PerceptionDBNodelet::cb_GetRTOVs, this);
            *_p_pdbreport_service = _p_nh->advertiseService("/perception/pdb/GetPDBReport", &PerceptionDBNodelet::cb_GetPDBReport, this);

        }

        //Initialize tf stuff

        //initialize parameters
        _name = _p_priv_nh->getNamespace();
        if (_verb) pp.info(std::ostringstream().flush() << "Namespace in PerceptionDBNodelet: " << _name.c_str());

        _p_priv_nh->param<bool>("verbose", _verb , false);
        _p_priv_nh->param<string>("db_path", _db_path , "/tmp/pdb");
        _p_priv_nh->param<std::string>("pdb_source", _pdb_source, "default_pdb");
        _p_priv_nh->param<bool>("from_scratch", _scratch , true);
        _p_priv_nh->param<bool>("write_mode", _w_mode , true);

        if (_pdb_source != "default_pdb" ) 
        {
            string system_call = "rosrun race_perception_utils pdb_source.sh " + _pdb_source;
            if ( system(system_call.c_str()) == 0 )
                pp.info(std::ostringstream().flush() << "success system_call to use pdb_source database : " << system_call.c_str());
            else
                pp.info(std::ostringstream().flush() << "fail system_call to use pdb_source database : " << system_call.c_str());
            
            _scratch = false;
        }

        //initialize the subscriber
        _p_pcin_subscriber = (boost::shared_ptr<ros::Subscriber>) new ros::Subscriber;


        //DB stuff
        this->connectDB();

        if (_flg_is_pdb)
        {

            //Output initialization information
            if (_verb) pp.info(std::ostringstream().flush() << _name.c_str() << ": OnInit finished. db_path is value: " << _db_path.c_str());
            pp.printInitialization();
        }
    }


    bool PerceptionDBNodelet::connectDB()
    {

        //DB stuff

        if (_flg_is_pdb )
        {
            _options.create_if_missing = true;
            //_options.error_if_exists = true;

            if (_scratch )
            {
                leveldb::Status status = leveldb::DestroyDB( _db_path, leveldb::Options()); // open the database
                ROS_INFO("Destroy DB to start from scratch!");
            }
            leveldb::Status status = leveldb::DB::Open(_options, _db_path, &_db); // open the database
            assert(status.ok());

            //status = _db->Put(leveldb::WriteOptions(), "A_1", "start of DB"); // Test Put
            //status = _db->Put(leveldb::WriteOptions(), "ZZZ_999", "end of DB"); // Test Put

            if (!status.ok()) cout << status.ToString() << endl;
            if (_verb) ROS_INFO("DB is opened in %s. Status is %s", _db_path.c_str(), status.ToString().c_str());

            if (_flg_is_nodelet )
            {

                _p_address_service = (boost::shared_ptr<ros::ServiceServer>) new ros::ServiceServer;

                *_p_address_service = _p_nh->advertiseService("/perception/pdb/get_db_address", &PerceptionDBNodelet::cb_get_db_address, this);
                if (_verb) NODELET_INFO("Start service /perception/pdb/get_db_address");
            }

        } 
        else if (_flg_is_nodelet)
        {
            if (_verb) ROS_INFO("connectDB at different nodelet: %s.", _name.c_str());

            //get db address by calling /get_db_address service
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::get_db_address>("/perception/pdb/get_db_address");

            race_perception_db::get_db_address srv;
            if (client.call(srv))
            {
                convertAddressStringToPointer(srv.response.mem_address, &_db);
                if (_verb) ROS_INFO("Address to db is %s. Creating local pointer to db (pointed to address %p)", srv.response.mem_address.c_str(), _db);
            }
            else
            {
                ROS_ERROR("%s: Failed to call service get_db_address", _name.c_str());
            }

        }

        return true;
    }


			/* _________________________________
			   |                                |
			   |     interface functions  		|
			   |________________________________| */

    bool PerceptionDBNodelet::put(const leveldb::Slice& key, const leveldb::Slice& value)
    {
        if (_verb) ROS_INFO("start put : key: %s, _flg_same_proc: %d", key.ToString().c_str(), _flg_same_proc);
        if (!_flg_same_proc)
        {
            //service call
            if (_verb) ROS_INFO("start service call in put as node : key: %s", key.ToString().c_str());
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::Put>("/perception/pdb/Put");

            race_perception_db::Put srv;
            srv.request.key.data.reserve(key.size());
            srv.request.key.data.assign(key.data(), key.data()+key.size());
            srv.request.key.size = key.size();
            srv.request.value.data.reserve(value.size());
            srv.request.value.data.assign(value.data(), value.data()+value.size());
            srv.request.value.size = value.size();
            if (client.call(srv))
            {
                if (_verb) ROS_INFO("Success to call service Put");
            }
            else
            {
                if (_verb) ROS_ERROR("Failed to call service Put");
                return false;
            }

        }
        else if (_w_mode)
        {
            leveldb::Status status = _db->Put(leveldb::WriteOptions(), key, value);
            if (_verb) ROS_INFO("start put as nodelet : key: %s", key.ToString().c_str());

            if (status.ok()) {
                if (_verb) ROS_INFO("put: key: %s, status: %s", key.ToString().c_str(), status.ToString().c_str());
            } else {
                if (_verb) ROS_INFO("put, status is not ok: key: %s, status: %s", key.ToString().c_str(), status.ToString().c_str());
                return false;
            }
        }
        return true;
    }

    bool PerceptionDBNodelet::del(const leveldb::Slice& key)
    {
        if (!_flg_same_proc)
        {
            //service call
            //if (_verb) ROS_INFO("start service call in del : key: %s", key.ToString().c_str());
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::Delete>("/perception/pdb/Delete");

            race_perception_db::Delete srv;
            srv.request.key.data.reserve(key.size());
            srv.request.key.data.assign(key.data(), key.data()+key.size());
            srv.request.key.size = key.size();
            if (client.call(srv))
            {
                if (_verb) ROS_INFO("Success to call service Delete");
            }
            else
            {
                ROS_ERROR("Failed to call service Delete");
                return false;
            }
        }
        else
        {
            leveldb::Status status = _db->Delete(leveldb::WriteOptions(), key);

            if (status.ok()) {
                if (_verb) ROS_INFO("del : key: %s, status: %s", key.ToString().c_str(), status.ToString().c_str());
            } else {
                if (_verb) ROS_INFO("del, status is not ok: key: %s, status: %s", key.ToString().c_str(), status.ToString().c_str());
                return false;
            }
        }
        return true;
    }

    bool PerceptionDBNodelet::get(const leveldb::Slice& key, string* value)
    {
        if (!_flg_same_proc)
        {
            //service call
            //if (_verb) ROS_INFO("start service call in get : key: %s", key.ToString().c_str());
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::Get>("/perception/pdb/Get");

            race_perception_db::Get srv;
            srv.request.key.data.reserve(key.size());
            srv.request.key.data.assign(key.data(), key.data()+key.size());
            srv.request.key.size = key.size();
            if (client.call(srv))
            {
                value->assign( srv.response.value.begin(), srv.response.value.end());
                //if (_verb) ROS_INFO("Success to call service Delete");
            }
            else
            {
                ROS_ERROR("Failed to call service Delete");
                return false;
            }
        }
        else
        {
            //if (_verb) NODELET_INFO("%s: get", _name.c_str());
            leveldb::Status status = _db->Get(leveldb::ReadOptions(), key, value);

            //if (_verb) NODELET_INFO("get:: key: %s, length:%u", key.ToString().c_str(), value->length());
        }
        return true;
    }

    string PerceptionDBNodelet::makeTopicName(const string t_name, const unsigned int nid)
    {  
        string topic(t_name);
        stringstream ss;
        ss << nid;
        topic += string(ss.str());

        return topic;
    }


			/* _________________________________
			   |                                |
			   |      PDB key interfaces  		|
			   |________________________________| */
    string PerceptionDBNodelet::makeSIKey(const string k_name, const unsigned int tid, const unsigned int vid, const unsigned int sid)
    {  
        string key(k_name);
        stringstream ss;
        ss << tid << SEP << vid << SEP << sid;
        key += string(ss.str());

        return key;
    }

    string PerceptionDBNodelet::makeKey(const string k_name, const unsigned int tid, const unsigned int vid)
    {  
        string key(k_name);
        stringstream ss;
        ss << tid << SEP << vid;
        key += string(ss.str());

        return key;
    }

    string PerceptionDBNodelet::makeKey(const string k_name, const unsigned int level, const unsigned int tid, const unsigned int vid)
    {  
        string key(k_name);
        stringstream ss;
        ss << level << SEP << tid << SEP << vid;
        key += string(ss.str());

        return key;
    }

    string PerceptionDBNodelet::makeOCKey(const string k_name, const string cat_name, const unsigned int cid)
    {  
        string key(k_name);
        stringstream ss;
        ss << cat_name << SEP << cid;
        key += string(ss.str());

        return key;
    }


			/* _________________________________
			   |                                |
			   |   levelDB query functions  	|
			   |________________________________| */
    vector <SITOV> PerceptionDBNodelet::querySITOVs(const string rtov_key)
    {
        vector <SITOV> v_si;

        //declare a new msg to be filled by the deserialization procedure
        RTOV rtov; 

        string value;
        _db->Get(leveldb::ReadOptions(), rtov_key, &value);

        uint32_t deserial_size = value.length(); 

        //Declare a shared array 
        boost::shared_array<uint8_t> buffer(new uint8_t[deserial_size]);
        memcpy(buffer.get(), value.data(), value.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

        //deserialize Msg 
        race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::deserialize(buffer, rtov, deserial_size);	
        for (uint32_t i=0; i< rtov.sitov_keys.size(); ++i)
        {
            SITOV sitov; 
            const string si_key = rtov.sitov_keys[i];
            string _si_value;
            _db->Get(leveldb::ReadOptions(), si_key, &_si_value);

            uint32_t si_size = _si_value.length(); 

            //Declare a shared array 
            boost::shared_array<uint8_t> si_buffer(new uint8_t[si_size]);
            memcpy(si_buffer.get(), _si_value.data(), _si_value.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

            //deserialize Msg 
            race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, SITOV>::deserialize(si_buffer, sitov, si_size);
            v_si.push_back(sitov);
            si_buffer.reset();
        }

        buffer.reset();
        return v_si;
    }

    vector <string> PerceptionDBNodelet::queryRTOVs(const unsigned int tid)
    {
        vector <string> v_vkey;

        //Get everythings from db
        leveldb::Iterator* it = _db->NewIterator(leveldb::ReadOptions());
        //for (it->SeekToFirst(); it->Valid(); it->Next()) {
        for (it->Seek(key::RV); it->Valid() ; it->Next()) {
            //if (_verb) ROS_INFO("%s:%s",it->key().ToString().c_str(), it->value().ToString().c_str());
            std::string _tmp_key(it->key().ToString());
            try {    
                if (!boost::starts_with(_tmp_key, key::RV))
                    break;

                //if (std::string::npos == _tmp_key.find(id_name))
                //    break;

                std::string _sub_key = _tmp_key.substr(key::RV.length(), key::RV.length());
                unsigned pos = _sub_key.find(SEP);
                std::string num = _sub_key.substr(0, pos);
                //unsigned int n = atoi(num.c_str());
                unsigned int n = boost::lexical_cast<unsigned int>(num);
                //if (_verb) ROS_INFO("num: \"%s\", n: %u, lid: %u", num.c_str(), n, lid);
                if (n == tid)
                    v_vkey.push_back(_tmp_key);
            } catch (boost::bad_lexical_cast) {
                // bad parameter
                ROS_INFO("bad_lexical_cast exception!!!!");
                continue;
            }

        }

        //if (_verb) ROS_INFO("Seek by key: %s, result key: %s, lid: %u", name.c_str(), _key.c_str(), lid);
        assert(it->status().ok());  // Check for any errors found during the scan
        delete it;

        return v_vkey;
    }

    //vector <string> PerceptionDBNodelet::queryPDBReport()
    MeasurePDB PerceptionDBNodelet::queryPDBReport()
    {
        //vector <string> v_report;
        MeasurePDB v_report;

        int n_pctov = 0;  
        int n_sitov = 0;  
        int n_rtov = 0;  
        int n_oc = 0;  
        int n_rrtov = 0;  
        
        v_report.pdb_size = 0;

        //Get everythings from db
        //leveldb::Slice start_key;
        //leveldb::Slice end_key;

        leveldb::Iterator* it = _db->NewIterator(leveldb::ReadOptions());
        it->SeekToFirst();

        //if (it->Valid())
            //start_key = it->key();

        for (; it->Valid(); it->Next()) {
            //for (it->Seek(key::RV); it->Valid() ; it->Next()) {
            string _tmp_key(it->key().ToString());
            //end_key = it->key();
            string value;
            _db->Get(leveldb::ReadOptions(), it->key(), &value);

            if(boost::starts_with(_tmp_key, key::PC)) {
                n_pctov++;
            } else if(boost::starts_with(_tmp_key, key::SI)) {
                n_sitov++; 
                v_report.si_size += value.length();
            } else if(boost::starts_with(_tmp_key, key::RV)) {
                n_rtov++; 
            } else if(boost::starts_with(_tmp_key, key::OC)) {
                n_oc++; 
            } else if(boost::starts_with(_tmp_key, key::RR)) {
                n_rrtov++; 
            }

            v_report.pdb_size += value.length();

            //Mike: for now lets just print all the keys
            //v_report.push_back(_tmp_key);
        }

        v_report.pctov = n_pctov / 1.0;
        v_report.sitov = n_sitov / 1.0;
        v_report.rtov = n_rtov / 1.0;
        v_report.oc = n_oc / 1.0;
        v_report.rrtov = n_rrtov / 1.0;

        //leveldb::Range range[1];
        //range[0] = leveldb::Range(start_key, end_key);
        ////range[0] = leveldb::Range("a", "z");
        ////range[0] = leveldb::Range("a", "Z");
        ////range[0] = leveldb::Range("A", "z");
        ////range[0] = leveldb::Range("A", "Z");
        //range[0] = leveldb::Range("A_1", "ZZZ_999");
        //uint64_t pdb_size[1];
        //_db->GetApproximateSizes(range, 1, pdb_size);
        //v_report.pdb_size = pdb_size[0];
        //ROS_INFO("report the size of pdb (1), start_key: %s ", start_key.ToString().c_str());
        //ROS_INFO("report the size of pdb (2), end_key: %s ", end_key.ToString().c_str());
        //ROS_INFO("report the size of pdb (3), pdb_size: %ld ", pdb_size[0]);
        //ROS_INFO("report the size of pdb (3), pdb_size: %ld ", pdb_size[1]);
        //ROS_INFO("report the size of pdb (3), pdb_size: %ld ", pdb_size[2]);
        //ROS_INFO("report the size of pdb (3), pdb_size: %ld ", pdb_size[3]);
        //ROS_INFO("report the size of pdb (3), pdb_size: %ld ", pdb_size[4]);
        //if (it->Valid())
        //{
            //start_key = it->key();
        
            //ROS_INFO("report the size of pdb (1), start key: %s ", start_key);
            //it->SeekToLast();
            //if (it->Valid())
            //{
                //end_key = it->key();
                //ROS_INFO("report the size of pdb (2), end key: %s ", end_key);
            //}
            //if ( start_key != end_key)
            //{
                //range[0] = leveldb::Range(start_key, end_key);
                //_db->GetApproximateSizes(range, 1, pdb_size);
                //ROS_INFO("report the size of pdb (3), pdb_size: %d ", pdb_size[0]);
                //v_report.pdb_size = pdb_size[0];
            //}
        //}
        //ROS_INFO("report the size of pdb: %d, # of feature: %d", v_report.pdb_size, n_sitov);
        //Mike: for now lets just print all the keys
        //string report("TrackedObjectPointCloud: ");
        //report += boost::lexical_cast<std::string>(n_pctov);
        //v_report.push_back(report);

        //report = "TrackedObjectFeature: ";
        //report += boost::lexical_cast<std::string>(n_sitov);
        //v_report.push_back(report);

        //report = "TrackedObjectView: ";
        //report += boost::lexical_cast<std::string>(n_rtov);
        //v_report.push_back(report);

        //report = "ObjectCategory: ";
        //report += boost::lexical_cast<std::string>(n_oc);
        //v_report.push_back(report);

        //report = "TrackedObjectRecognition: ";
        //report += boost::lexical_cast<std::string>(n_rrtov);
        //v_report.push_back(report);

        assert(it->status().ok());  // Check for any errors found during the scan
        delete it;

        return v_report;
    }

    RTOV PerceptionDBNodelet::queryRTOV(const string view_key)
    {
        //bool ret;

        leveldb::Slice key(view_key);

        RTOV view;
        //const string si_key = rtov.sitov_keys[i];
        string view_value;
        _db->Get(leveldb::ReadOptions(), key, &view_value);

        uint32_t view_size = view_value.length(); 

        //Declare a shared array 
        boost::shared_array<uint8_t> view_buffer(new uint8_t[view_size]);
        memcpy(view_buffer.get(), view_value.data(), view_value.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

        //deserialize Msg 
        race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::deserialize(view_buffer, view, view_size);

        return view;
 
    }
 
    RTOV PerceptionDBNodelet::getRTOV(const string key)
    {
        RTOV view;

        //if (_verb) ROS_INFO("called getRTOV: key: %s", key.c_str());

        if (!_flg_same_proc)
        {
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::GetRTOV>("/perception/pdb/GetRTOV");

            race_perception_db::GetRTOV srv;
            srv.request.key = key;

            if (client.call(srv))
            {
                //if (_verb) ROS_INFO("Success to call service getRTOVs");
                view = srv.response.rtov;
            }
            else
            {
                ROS_ERROR("Failed to call service getRTOV");
                return view;
            }
        }
        else
        {
            view = this->queryRTOV(key);
        }

        return view;
    }

   
    vector <string> PerceptionDBNodelet::getRTOVs(const unsigned int tid)
    {
        vector <string> v_vkey;

        //if (_verb) ROS_INFO("called getRTOVs: tid: %u", tid);

        if (!_flg_same_proc)
        {
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::GetRTOVs>("/perception/pdb/GetRTOVs");

            race_perception_db::GetRTOVs srv;
            srv.request.tid = tid;

            if (client.call(srv))
            {
                //if (_verb) ROS_INFO("Success to call service getRTOVs");
                v_vkey = srv.response.rtov_key;
            }
            else
            {
                ROS_ERROR("Failed to call service getRTOVs");
                return v_vkey;
            }
        }
        else
        {
            v_vkey = this->queryRTOVs(tid);
        }

        return v_vkey;
    }

    vector <SITOV> PerceptionDBNodelet::getSITOVs(const string rtov_key)
    {
        vector <SITOV> v_si;

        //if (_verb) ROS_INFO("called getSITOVs: rtov_key: %s", rtov_key.c_str());

        if (!_flg_same_proc)
        {
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::GetSITOVs>("/perception/pdb/GetSITOVs");

            race_perception_db::GetSITOVs srv;
            srv.request.rtov_key.reserve(rtov_key.length());
            srv.request.rtov_key.assign(rtov_key); //, name+name.length());

            if (client.call(srv))
            {
                //if (_verb) ROS_INFO("Success to call service GetSITOVs");
                v_si = srv.response.sitovs;
            }
            else
            {
                ROS_ERROR("Failed to call service GetSITOVs");
                return v_si;
            }
        }
        else
        {
            v_si = this->querySITOVs(rtov_key);
        }

        return v_si;
    }

    vector <ObjectCategory> PerceptionDBNodelet::queryAllObjectCat()
    {
        vector <ObjectCategory> v_oc;

        leveldb::Iterator* it = _db->NewIterator(leveldb::ReadOptions());

        for (it->Seek(key::OC); it->Valid() ; it->Next()) {
            //if (_verb) ROS_INFO("%s:%s",it->key().ToString().c_str(), it->value().ToString().c_str());
            string _tmp_key(it->key().ToString());
            try {    
                if (!boost::starts_with(_tmp_key, key::OC))
                    break;

                ObjectCategory oc;
                //const string si_key = rtov.sitov_keys[i];
                string oc_value;
                _db->Get(leveldb::ReadOptions(), it->key(), &oc_value);

                uint32_t oc_size = oc_value.length(); 

                //Declare a shared array 
                boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
                memcpy(oc_buffer.get(), oc_value.data(), oc_value.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

                //deserialize Msg 
                race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::deserialize(oc_buffer, oc, oc_size);
                v_oc.push_back(oc);
                oc_buffer.reset();
            } catch (boost::bad_lexical_cast) {
                // bad parameter
                ROS_INFO("bad_lexical_cast exception!!!!");
                continue;
            }

        }

        //if (_verb) ROS_INFO("Seek by key: %s, result key: %s, lid: %u", name.c_str(), _key.c_str(), lid);
        assert(it->status().ok());  // Check for any errors found during the scan
        delete it;

        return v_oc;
    }

    vector <string> PerceptionDBNodelet::queryKeys(const string k_name)
    {
        vector <string> v_key;

        leveldb::Iterator* it = _db->NewIterator(leveldb::ReadOptions());

        for (it->Seek(k_name); it->Valid() ; it->Next()) {

            string _tmp_key(it->key().ToString());
            try {    
                if (!boost::starts_with(_tmp_key, k_name))
                    break;
                v_key.push_back(_tmp_key);
            } catch (boost::bad_lexical_cast) {
                // bad parameter
                ROS_INFO("bad_lexical_cast exception!!!!");
                continue;
            }
        }

        return v_key;
    }

    vector <ObjectCategory> PerceptionDBNodelet::getAllObjectCat()
    {
        vector <ObjectCategory> v_oc;
        //if (_verb) ROS_INFO("called getAllObjectCat");

        if (!_flg_same_proc)
        {
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::GetAllObjectCat>("/perception/pdb/GetAllObjectCat");

            race_perception_db::GetAllObjectCat srv;
            //srv.request.rtov_key.reserve(rtov_key.length());
            //srv.request.rtov_key.assign(rtov_key); //, name+name.length());

            if (client.call(srv))
            {
                if (_verb) ROS_INFO("Success to call service GetAllObjectCat");
                v_oc = srv.response.ocs;
            }
            else
            {
                ROS_ERROR("Failed to call service GetAllObjectCat");
                return v_oc;
            }
        }
        else
        {
            v_oc = this->queryAllObjectCat();
        }


        return v_oc;
    }


    vector <string> PerceptionDBNodelet::getKeys(const std::string k_name)
    {
        vector <string> v_key;
        //if (_verb) ROS_INFO("called getAllObjectCat");

        if (!_flg_same_proc)
        {
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::GetKeys>("/perception/pdb/GetKeys");

            race_perception_db::GetKeys srv;
            srv.request.k_name.reserve(k_name.length());
            srv.request.k_name.assign(k_name); //, name+name.length());

            if (client.call(srv))
            {
                if (_verb) ROS_INFO("Success to call service GetAllObjectCat");
                v_key = srv.response.keys;
            }
            else
            {
                ROS_ERROR("Failed to call service GetAllObjectCat");
                return v_key;
            }
        }
        else
        {
            v_key = this->queryKeys(k_name);
        }


        return v_key;
    }

    //vector <string> PerceptionDBNodelet::reportPDB()
    MeasurePDB PerceptionDBNodelet::reportPDB()
    {
        //vector <string> v_report;
        MeasurePDB v_report;


        //if (_verb) ROS_INFO("called reportPDB");

        if (!_flg_same_proc)
        {
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::GetPDBReport>("/perception/pdb/GetPDBReport");

            race_perception_db::GetPDBReport srv;

            if (client.call(srv))
            {
                //if (_verb) ROS_INFO("Success to call service GetPDBReport");
                v_report = srv.response.report;
            }
            else
            {
                ROS_ERROR("Failed to call service GetPDBReport");
                return v_report;
            }
        }
        else
        {
            v_report = this->queryPDBReport();
        }


        v_report.header.stamp = ros::Time::now();
        //_p_pdbreport_publisher->publish(v_report);

        return v_report;
    }

    unsigned int PerceptionDBNodelet::getLastTrkID()
    {
        //if (_verb) ROS_INFO("called getLastTrkID");

        unsigned int trkid = getLastNodeID(key::RV, SEP);

        return trkid;

    }

    unsigned int PerceptionDBNodelet::getLastNodeID(const string name, const string id_name = SEP)
    {
        unsigned int lid = 0;

        //if (_verb) ROS_INFO("called getLastNodeID : %s, and: %s", name.c_str(), id_name.c_str());

        if (!_flg_same_proc)
        {
            //service call
            //Config service clients
            ros::ServiceClient client = _p_nh->serviceClient<race_perception_db::LastNodeID>("/perception/pdb/LastNodeID");

            race_perception_db::LastNodeID srv;
            srv.request.name.reserve(name.length());
            srv.request.name.assign(name); //, name+name.length());
            srv.request.id_name.reserve(id_name.length());
            srv.request.id_name.assign(id_name); //, id_name+id_name.length());

            //if (_verb) ROS_INFO("call service LastNodeID with request.name:%s, request.id_name:%s", srv.request.name.c_str(), srv.request.id_name.c_str());
            if (client.call(srv))
            {
                //if (_verb) ROS_INFO("Success to call service LastNodeID");
                lid = srv.response.nid;
            }
            else
            {
                if (_verb) ROS_INFO("Try again to get LastNodeID");
                return lid;
            }
        }
        else
        {
            //if (_verb) ROS_INFO("access DB to get LastNodeID : %s, id_name: %s", name.c_str(), id_name.c_str());
            //Get everythings from db
            leveldb::Iterator* it = _db->NewIterator(leveldb::ReadOptions());
            string _key;
            //for (it->SeekToFirst(); it->Valid(); it->Next()) {
            for (it->Seek(name); it->Valid() ; it->Next()) {
                //if (_verb) ROS_INFO("%s:%s",it->key().ToString().c_str(), it->value().ToString().c_str());
                string _tmp_key(it->key().ToString());
                try {    
                    if (!boost::starts_with(_tmp_key, name))
                        break;

                    if (string::npos == _tmp_key.find(id_name))
                        break;

                    //_key = it->key().ToString();
                    _key = _tmp_key;
                    string _sub_key = _key.substr(name.length(), name.length());
                    unsigned pos = _sub_key.find(id_name);
                    string num = _sub_key.substr(0, pos);
                    //unsigned int n = atoi(num.c_str());
                    unsigned int n = boost::lexical_cast<unsigned int>(num);
                    //if (_verb) ROS_INFO("num: \"%s\", n: %u, lid: %u", num.c_str(), n, lid);
                    if (n > lid)
                        lid = n;
                } catch (boost::bad_lexical_cast) {
                    // bad parameter
                    ROS_INFO("bad_lexical_cast exception!!!!");
                    continue;
                }

            }

            //if (_verb) ROS_INFO("Seek by key: %s, result key: %s, lid: %u", name.c_str(), _key.c_str(), lid);
            assert(it->status().ok());  // Check for any errors found during the scan
            delete it;
        }
        //if (_verb) ROS_INFO("%s: result of getLastNodeID lid:%u", _name.c_str(), lid);
        return lid;
    }

};
