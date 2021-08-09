#define PFLN printf("LINE %d FILE %s\n",__LINE__, __FILE__);

#include <ros/ros.h>

#include <stdio.h>


#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>
#include <race_perception_msgs/perception_msgs.h>

#include <race_perception_utils/print.h>
#include <race_perception_utils/cycle.h>

using namespace std;
using namespace race_perception_db;
using namespace race_perception_msgs;
using namespace race_perception_utils;

/* _________________________________
   |                                 |
   |        GLOBAL VARIABLES         |
   |_________________________________| */

boost::shared_ptr<ros::Publisher> _p_publisher;


std::string _name="pdb_tutorial";
PerceptionDB* _pdb; //initialize the class

void add_msg(const unsigned int cid){
    //SITOV msg_in;
    ObjectCategoryBayesModel _obm;
    _obm.cat_id = cid;
    _obm.cat_name = "objBayesModel";

    uint32_t buf_size = ros::serialization::serializationLength(_obm);

    boost::shared_array<uint8_t> buffer(new uint8_t[buf_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategoryBayesModel>::serialize(buffer, _obm, buf_size);
    leveldb::Slice slice((char*)buffer.get(), buf_size);
    //string PerceptionDBNodelet::makeKey(const string k_name, const unsigned int tid, const unsigned int vid)
    std::string obm_key = _pdb->makeKey(key::OBM, cid, 1 );

    //Put slice to the db
    _pdb->put(obm_key, slice); 
}
	
	
	
void add_TOVI_msg(const unsigned int cid){
    //SITOV msg_in;
    TOVI _tovi;
    _tovi.track_id = cid;
    _tovi.view_id = 1;
    _tovi.pose_stamped.pose.position.x = 0.1;
    _tovi.pose_stamped.pose.position.y = 0.1;
    _tovi.pose_stamped.pose.position.z = 0.1;
    _tovi.object_label = "test";
    _tovi.minimum_distance =  0.1;
    
    uint32_t buf_size = ros::serialization::serializationLength(_tovi);

    boost::shared_array<uint8_t> buffer(new uint8_t[buf_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, TOVI>::serialize(buffer, _tovi, buf_size);
    leveldb::Slice slice((char*)buffer.get(), buf_size);
    //string PerceptionDBNodelet::makeKey(const string k_name, const unsigned int tid, const unsigned int vid)
    std::string tovi_key = _pdb->makeKey(key::TOVI, cid, 1 );

    //Put slice to the db
    _pdb->put(tovi_key, slice); 
}
	
void timer_callback(const ros::TimerEvent& input)
{
    //ROS_INFO("timer_callback started!");
    PrettyPrint pp(_name);
    pp.info(std::ostringstream().flush() << "timer_callback started!");

	int _trackid_count = _pdb->getLastTrkID();
    pp.info(std::ostringstream().flush() << "LastTrkID " << _trackid_count);

    static unsigned int cid = 1;
    add_msg(cid++);

    //vector <string> keys = _pdb->getKeys(key::RV);
    vector <string> keys = _pdb->getKeys(key::OBM);
    //pp.info(std::ostringstream().flush() << "getKeys( " << key::RV << " ) ");
    pp.info(std::ostringstream().flush() << "getKeys( " << key::OBM << " ) ");

    //vector <RTOV> rtovs;
    vector <ObjectCategoryBayesModel> obms;
    for ( vector<string>::iterator it = keys.begin(); it !=keys.end(); ++it)
    {
        pp.info(std::ostringstream().flush() << "key: " << *it);

        //RTOV rtov; 
        ObjectCategoryBayesModel obm; 
        string value;
        _pdb->get(*it, &value);

        uint32_t deserial_size = value.length();
        pp.info(std::ostringstream().flush() << "get:: value.length():" << deserial_size);
        boost::shared_array<uint8_t> buffer(new uint8_t[deserial_size]);
        memcpy(buffer.get(), value.data(), deserial_size);

        //race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::deserialize(buffer, rtov, deserial_size);
        race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategoryBayesModel>::deserialize(buffer, obm, deserial_size);

        //pp.info(std::ostringstream().flush() << "get:: track_id " << rtov.track_id << ": view_id " << rtov.view_id);
        //rtovs.push_back(rtov);
        pp.info(std::ostringstream().flush() << "get:: cat_id " << obm.cat_id );
        obms.push_back(obm);

    }

    //pp.info(std::ostringstream().flush() << "rtovs.size(): " << rtovs.size());
    pp.info(std::ostringstream().flush() << "obms.size(): " << obms.size());
    pp.printCallback();

}


void timer_callback_tovi(const ros::TimerEvent& input)
{
    //ROS_INFO("timer_callback started!");
    PrettyPrint pp(_name);
    pp.info(std::ostringstream().flush() << "timer_callback_tovi started!");

	int _trackid_count = _pdb->getLastTrkID();
    pp.info(std::ostringstream().flush() << "LastTrkID " << _trackid_count);

    static unsigned int cid = 1;
    add_TOVI_msg(cid++);

    //vector <string> keys = _pdb->getKeys(key::RV);
    vector <string> keys = _pdb->getKeys(key::TOVI);
    //pp.info(std::ostringstream().flush() << "getKeys( " << key::RV << " ) ");
    pp.info(std::ostringstream().flush() << "getKeys( " << key::TOVI << " ) ");

    //vector <RTOV> rtovs;
    vector <TOVI> tovis;
    for ( vector<string>::iterator it = keys.begin(); it !=keys.end(); ++it)
    {
        pp.info(std::ostringstream().flush() << "key: " << *it);

        //RTOV rtov; 
        TOVI _tovi; 
        string value;
        _pdb->get(*it, &value);

        uint32_t deserial_size = value.length();
        pp.info(std::ostringstream().flush() << "get:: value.length():" << deserial_size);
        boost::shared_array<uint8_t> buffer(new uint8_t[deserial_size]);
        memcpy(buffer.get(), value.data(), deserial_size);

        //race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::deserialize(buffer, rtov, deserial_size);
        race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, TOVI>::deserialize(buffer, _tovi, deserial_size);

        //pp.info(std::ostringstream().flush() << "get:: track_id " << rtov.track_id << ": view_id " << rtov.view_id);
        //rtovs.push_back(rtov);
        pp.info(std::ostringstream().flush() << "get:: track_id " << _tovi.track_id );
        tovis.push_back(_tovi);

    }

    //pp.info(std::ostringstream().flush() << "rtovs.size(): " << rtovs.size());
    pp.info(std::ostringstream().flush() << "tovis.size(): " << tovis.size());
    pp.printCallback();

}



void cycle_callback(const race_perception_utils::Cycle& msg)
{
    ROS_INFO("cycle_callback started!");
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, _name); // Initialize ROS coms
    ros::NodeHandle* _p_nh;
    _p_nh = (ros::NodeHandle*) new ros::NodeHandle(_name); //The node handle
    _name = _p_nh->getNamespace();

    _pdb = race_perception_db::PerceptionDB::getPerceptionDB(_p_nh); //initialize the class
    //ros::Subscriber _subscriber = _p_nh->subscribe ("/perception/cycle", 10, cycle_callback);
    ros::Timer _timer = _p_nh->createTimer(ros::Duration(1.0), timer_callback_tovi);

    PrettyPrint pp(_name);
    pp.printInitialization();
    ros::spin();

    return 1;
}

