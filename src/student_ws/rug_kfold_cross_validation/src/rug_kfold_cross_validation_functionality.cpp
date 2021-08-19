#ifndef _KFOLD_CROSS_VALIDATION_LIB_CPP_
#define _KFOLD_CROSS_VALIDATION_LIB_CPP_

#define recognitionThershold 2


/* _________________________________
   |                                 |
   |           INCLUDES              |
   |_________________________________| */

//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>

//ros includes 
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/ply_io.h>

//RACE includes
#include <race_perception_msgs/perception_msgs.h>
#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>
#include <object_descriptor/object_descriptor_functionality.h>
#include <object_conceptualizer/object_conceptualization.h>
#include <feature_extraction/spin_image.h>
#include <race_perception_utils/cycle.h>
#include <race_perception_utils/print.h>

//pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h>
#include <pcl/features/gfpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/grsd.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <sys/time.h>  
#include <iostream>
#include <dirent.h>  //I use dirent.h for get folders in directory which is also available for windows:

// define deep learning service here
#include <rug_deep_feature_extraction/deep_representation.h>
// #include <race_deep_learning_feature_extraction/deep_representation.h>

#include <math.h>   /* atan2 */
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

/*_________________________________
|                                 |
|         Global variable         |
|_________________________________| */
 
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBA T;  
PerceptionDB* _pdb; //initialize the class

int track_id_gloabal =1;
using namespace pcl;
using namespace std;
using namespace ros;

//////////////////////////////////////////////////////////////////////////////////////////////////
struct KNN_struct{
    vector <float> distances;
    string category_name;
};

//////////////////////////////////////////////////////////////////////
int kFoldTrainAndTestDataGenerator( int K_fold, 
                                    int iteration, 
                                    vector<vector<SITOV> > categories_instances,
                                    vector<vector<SITOV> > &categories_instances_train,
                                    vector<vector<SITOV> > &categories_instances_test)
{
    /// iteration [0-9]. K_fold = 10
    ROS_INFO(" ----------- fold%d -----------", iteration+1);
    for (size_t i = 0; i < categories_instances.size(); i++) // retrieves all categories 
    {                           
        
        int category_size = categories_instances.at(i).size();
        ROS_INFO("category_size = %d", category_size);

        int left_idx = 0, right_idx =0;

        left_idx = int(category_size/K_fold) * (iteration);
        
        ROS_INFO("left_idx = %d", left_idx);

        if (iteration != K_fold-1) 
        {
            right_idx = int(category_size/K_fold) * (iteration + 1);
        }
        else
        {
            right_idx = categories_instances.at(i).size();
        }
        ROS_INFO("right_idx = %d", right_idx);

        vector<SITOV> train_tmp;
        vector<SITOV> test_tmp;
        for (int j = 0; j < categories_instances.at(i).size(); j++)
        {
            if ((j >= left_idx) && (j < right_idx))   
            {
                test_tmp.push_back(categories_instances.at(i).at(j));
                // std::cout << "name test = " << categories_instances.at(i).at(j).ground_truth_name << endl; 
            }
            else
            {
                train_tmp.push_back(categories_instances.at(i).at(j));
                // std::cout << "name train = " << categories_instances.at(i).at(j).ground_truth_name << endl; 
            }
        }
        ROS_INFO("---------------------------------");

        categories_instances_train.push_back(train_tmp);
        categories_instances_test.push_back(test_tmp);
    
    }
        
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////
void multiplyVectorElementsByAScalarValue ( vector <float> &vector_list, double weight)
{
    for (int i = 0; i < vector_list.size(); i++ )
    {
        vector_list.at(i) =  weight *  vector_list.at(i);
    }
}

///////////////////////////////////////////////////////////////////////////////////////
vector <float> normalization (vector <float> hist)
{
    // sum of all elements
    float sum_of_elems = 0;
    for(std::vector<float>::iterator it = hist.begin(); it != hist.end(); ++it)
        sum_of_elems += *it;

    //normalizing histogram.
    for (size_t i = 0; i < hist.size(); i++)
    {
		hist.at(i) /= sum_of_elems;
  	}
    return (hist);  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
string extractObjectName (string object_name_orginal )
{
    std:: string object_name;
    std:: string tmp_object_name = object_name_orginal;// for writing object name in result file;
    int rfind = tmp_object_name.rfind("//")+2;       
    int len = tmp_object_name.length();

    object_name = object_name_orginal.substr(rfind, tmp_object_name.length());

	while (object_name.length() < 30)
    { 
		object_name += " ";
    }

    return (object_name);
} 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
string extractFileName (string object_name_orginal )
{
    std:: string object_name;
    std:: string tmp_object_name = object_name_orginal;// for writing object name in result file;
    int rfind = tmp_object_name.rfind("//")+2;       
    int len = tmp_object_name.length()-4; // -4 for removeing file format .pcd or .ply

    object_name = object_name_orginal.substr(rfind, len); 
	ROS_INFO("object_name = %s", object_name.c_str());
    return (object_name);
} 

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
string extractCategoryName (string instance_path )
{
    string category_name="";	    
    int ffind = instance_path.find("//")+2;  
	int lfind =  instance_path.find("_Cat");       
     
    for (int i=0; i<(lfind-ffind); i++)
    {
		category_name += instance_path.at(i+ffind);
    }
    return (category_name);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int estimateESFDescription (boost::shared_ptr<PointCloud<PointT> > cloud, 
			     			pcl::PointCloud<pcl::ESFSignature640>::Ptr &descriptor)
{
    // ESF estimation object.
    pcl::ESFEstimation<PointT, pcl::ESFSignature640> esf;
    esf.setInputCloud(cloud);
    esf.compute(*descriptor);

    return 0;  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int addObjectViewHistogramInSpecificCategory(	std::string cat_name, unsigned int cat_id, 
												unsigned int track_id, unsigned int view_id, 
												SITOV objectViewHistogram , PrettyPrint &pp
												)
{
	//// add representation of an object to perceptual memory
    SITOV msg_in;
    RTOV _rtov;
    _rtov.track_id = track_id;
    _rtov.view_id = view_id;
	msg_in = objectViewHistogram;
	msg_in.spin_img_id = 1;

	ros::Time start_time = ros::Time::now();
	uint32_t sp_size = ros::serialization::serializationLength(msg_in);

	boost::shared_array<uint8_t> sp_buffer(new uint8_t[sp_size]);
	PerceptionDBSerializer<boost::shared_array<uint8_t>, SITOV>::serialize(sp_buffer, msg_in, sp_size);
	leveldb::Slice sp_s((char*)sp_buffer.get(), sp_size);
	std::string sp_key = _pdb->makeSIKey(key::SI, track_id, view_id, 1 );

	//// put slice to the db
	_pdb->put(sp_key, sp_s); 

	//ROS_INFO("****========***** add a FEATURE to PDB took = %f", (ros::Time::now() - start_time).toSec());

	//// ADD A NEW OBJECT VIEW TO PDB
	start_time = ros::Time::now();

	//create a list of key of spinimage
	_rtov.sitov_keys.push_back(sp_key);

    uint32_t v_size = ros::serialization::serializationLength(_rtov);

    boost::shared_array<uint8_t> v_buffer(new uint8_t[v_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::serialize(v_buffer, _rtov, v_size);	

    leveldb::Slice v_s((char*)v_buffer.get(), v_size);

    std::string v_key = _pdb->makeKey(key::RV, track_id, view_id);
    //ROS_INFO("\t\t[-]v_key: %s, view_id: %i, track_id: %i", v_key.c_str(), view_id, track_id);

    //// Put one view to the db
    _pdb->put(v_key, v_s);
	//ROS_INFO("****========***** add an object view to PDB took = %f", (ros::Time::now() - start_time).toSec());

	//// UDPDATE CATEGORY TO PDB
	start_time = ros::Time::now();
    ObjectCategory _oc;
    std::string oc_key = _pdb->makeOCKey(key::OC, cat_name, cat_id);
    std::string str_oc;
    _pdb->get(oc_key, &str_oc);
    uint32_t oc_size = str_oc.length();
    if (oc_size != 0) //Object category exist.
    {
        boost::shared_array<uint8_t> oc_dbuffer(new uint8_t[oc_size]);
        memcpy(oc_dbuffer.get(), str_oc.data(), str_oc.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

        //deserialize Msg 
        race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, race_perception_msgs::ObjectCategory>::deserialize(oc_dbuffer, _oc, oc_size);
    }

    _oc.cat_name = cat_name;
    _oc.cat_id = cat_id ;
    _oc.rtov_keys.push_back(v_key);
    _oc.icd = 1.0f;
    oc_size = ros::serialization::serializationLength(_oc);

    //pp.info(std::ostringstream().flush() << _oc.cat_name.c_str() << " category has " << _oc.rtov_keys.size() << " objects.");

    boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::serialize(oc_buffer, _oc, oc_size);	
    leveldb::Slice ocs((char*)oc_buffer.get(), oc_size);
    _pdb->put(oc_key, ocs);
	//ROS_INFO("****========***** update a category took = %f", (ros::Time::now() - start_time).toSec());
    return (1);
}


int addObjectViewHistogramInSpecificCategoryDeepLearning(std::string cat_name, unsigned int cat_id, 
															unsigned int track_id, unsigned int view_id, 
															SITOV objectViewHistogram , PrettyPrint &pp
															)
{
	//// ADD A NEW OBJECT Feature TO PDB
    SITOV msg_in;
    RTOV _rtov;
    _rtov.track_id = track_id;
    _rtov.view_id = view_id;

	msg_in = objectViewHistogram;
	msg_in.spin_img_id = 1;


	ros::Time start_time = ros::Time::now();

	uint32_t sp_size = ros::serialization::serializationLength(msg_in);

	boost::shared_array<uint8_t> sp_buffer(new uint8_t[sp_size]);
	PerceptionDBSerializer<boost::shared_array<uint8_t>, SITOV>::serialize(sp_buffer, msg_in, sp_size);
	leveldb::Slice sp_s((char*)sp_buffer.get(), sp_size);
	std::string sp_key = _pdb->makeSIKey(key::SI, track_id, view_id, 1 );

	//Put slice to the db
	_pdb->put(sp_key, sp_s); 

	//ROS_INFO("****========***** add a FEATURE to PDB took = %f", (ros::Time::now() - start_time).toSec());


	//// ADD A NEW OBJECT VIEW TO PDB
	start_time = ros::Time::now();

	//create a list of key of spinimage
	_rtov.sitov_keys.push_back(sp_key);

    uint32_t v_size = ros::serialization::serializationLength(_rtov);

    boost::shared_array<uint8_t> v_buffer(new uint8_t[v_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::serialize(v_buffer, _rtov, v_size);	

    leveldb::Slice v_s((char*)v_buffer.get(), v_size);

    std::string v_key = _pdb->makeKey(key::RV, track_id, view_id);
    ROS_INFO("\t\t[-] v_key: %s, view_id: %i, track_id: %i", v_key.c_str(), view_id, track_id);

    //Put one view to the db
    _pdb->put(v_key, v_s);
	//ROS_INFO("****========***** add a VIEW to PDB took = %f", (ros::Time::now() - start_time).toSec());


	//// UDPDATE CATEGORY TO PDB
	start_time = ros::Time::now();

    ObjectCategory _oc;

    std::string oc_key = _pdb->makeOCKey(key::OC, cat_name, cat_id);

    std::string str_oc;
    _pdb->get(oc_key, &str_oc);
    uint32_t oc_size = str_oc.length();
    if (oc_size != 0) //Object category exist.
    {
        boost::shared_array<uint8_t> oc_dbuffer(new uint8_t[oc_size]);
        memcpy(oc_dbuffer.get(), str_oc.data(), str_oc.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

        //deserialize Msg 
        race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, race_perception_msgs::ObjectCategory>::deserialize(oc_dbuffer, _oc, oc_size);
    }

    _oc.cat_name = cat_name;
    _oc.cat_id = cat_id ;
    _oc.rtov_keys.push_back(v_key);

    // when a new object view add to database, ICD should be update
//     vector <  vector <SITOV> > category_instances;
//     for (size_t i = 0; i < _oc.rtov_keys.size(); i++)
//     {
//         vector <SITOV> objectViewSpinimages = _pdb->getSITOVs(_oc.rtov_keys.at(i).c_str());
//         category_instances.push_back(objectViewSpinimages);
//     }

    // std::vector< SITOV > category_instances; 
    // for (size_t i = 0; i < _oc.rtov_keys.size(); i++)
    // {
	// 	vector< SITOV > objectViewHistogram = _pdb->getSITOVs(_oc.rtov_keys.at(i).c_str());
	// 	category_instances.push_back(objectViewHistogram.at(0));
	// 	// pp.info(std::ostringstream().flush() << "size of object view histogram = " << objectViewHistogram.size());
	// 	// pp.info(std::ostringstream().flush() << "key for the object view histogram = " << v_oc.at(i).rtov_keys.at(j).c_str());
    // }

    
    float New_ICD = 1;
//     intraCategoryDistance(category_instances, New_ICD, pp);
    //histogramBasedIntraCategoryDistance(category_instances,New_ICD,pp);
    _oc.icd = New_ICD;

    oc_size = ros::serialization::serializationLength(_oc);

    pp.info(std::ostringstream().flush() << _oc.cat_name.c_str() << " category has " << _oc.rtov_keys.size() << " objects.");
    pp.info(std::ostringstream().flush() << "ICD for " << _oc.cat_name.c_str() << " category updated. New ICD is: "<< _oc.icd);

    boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::serialize(oc_buffer, _oc, oc_size);	
    leveldb::Slice ocs((char*)oc_buffer.get(), oc_size);
    _pdb->put(oc_key, ocs);
	//ROS_INFO("****========***** update a CATEGORY to PDB took = %f", (ros::Time::now() - start_time).toSec());
	//ROS_INFO("****========*********========*********========*********========*********========*****");
    return (1);
}

int conceptualizeObjectViewSpinImagesInSpecificCategory(std::string cat_name, unsigned int cat_id, 
														unsigned int track_id, unsigned int view_id, 
														vector <SITOV> SpinImageMsg , PrettyPrint &pp
														)
{
    //PrettyPrint pp;
    SITOV msg_in;
    RTOV _rtov;
    _rtov.track_id = track_id;
    _rtov.view_id = view_id;

    for (size_t i = 0; i < SpinImageMsg.size(); i++)
    {
        msg_in = SpinImageMsg.at(i);
        msg_in.spin_img_id = i;

        uint32_t sp_size = ros::serialization::serializationLength(msg_in);

        boost::shared_array<uint8_t> sp_buffer(new uint8_t[sp_size]);
        PerceptionDBSerializer<boost::shared_array<uint8_t>, SITOV>::serialize(sp_buffer, msg_in, sp_size);
        leveldb::Slice sp_s((char*)sp_buffer.get(), sp_size);
        std::string sp_key = _pdb->makeSIKey(key::SI, track_id, view_id, i );

        //Put slice to the db
        _pdb->put(sp_key, sp_s); 

        //create a list of key of spinimage
        _rtov.sitov_keys.push_back(sp_key);

    }

    uint32_t v_size = ros::serialization::serializationLength(_rtov);

    boost::shared_array<uint8_t> v_buffer(new uint8_t[v_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, RTOV>::serialize(v_buffer, _rtov, v_size);	

    leveldb::Slice v_s((char*)v_buffer.get(), v_size);

    std::string v_key = _pdb->makeKey(key::RV, track_id, view_id);
    ROS_INFO("\t\t[-]v_key: %s, view_id: %i, track_id: %i", v_key.c_str(), view_id, track_id);

    //Put one view to the db
    _pdb->put(v_key, v_s);

    ObjectCategory _oc;

    std::string oc_key = _pdb->makeOCKey(key::OC, cat_name, cat_id);

    std::string str_oc;
    _pdb->get(oc_key, &str_oc);
    
    uint32_t oc_size = str_oc.length();
    if (oc_size != 0) //Object category exist.
    {
        boost::shared_array<uint8_t> oc_dbuffer(new uint8_t[oc_size]);
        memcpy(oc_dbuffer.get(), str_oc.data(), str_oc.size()); //Maybe a bug in ROS:: without this memcpy, runtime error occurs!!!!!!

        //deserialize Msg 
        race_perception_db::PerceptionDBSerializer<boost::shared_array<uint8_t>, race_perception_msgs::ObjectCategory>::deserialize(oc_dbuffer, _oc, oc_size);
    }

    _oc.cat_name = cat_name;
    _oc.cat_id = cat_id ;
    _oc.rtov_keys.push_back(v_key);

    // when a new object view add to database, ICD should be update
    vector <  vector <SITOV> > category_instances;
    for (size_t i = 0; i < _oc.rtov_keys.size(); i++)
    {
		vector <SITOV> objectViewSpinimages = _pdb->getSITOVs(_oc.rtov_keys.at(i).c_str());
        category_instances.push_back(objectViewSpinimages);
    }

    ROS_INFO("OC %s has %d instances", _oc.cat_name.c_str(), _oc.rtov_keys.size());
    //float New_ICD = 0;
    
    intraCategoryDistance(category_instances, _oc.icd, pp);
    //_oc.icd = New_ICD;

    oc_size = ros::serialization::serializationLength(_oc);

    pp.info(std::ostringstream().flush() << _oc.cat_name.c_str() << " category has " << _oc.rtov_keys.size() << " objects.");
    pp.info(std::ostringstream().flush() << "ICD for " << _oc.cat_name.c_str() << " category updated. New ICD is: "<< _oc.icd);

    boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::serialize(oc_buffer, _oc, oc_size);	
    leveldb::Slice ocs((char*)oc_buffer.get(), oc_size);
    _pdb->put(oc_key, ocs);

    return (1);
}

void delelteAllRVFromDB ()
{
	vector <string> RVkeys = _pdb->getKeys(key::RV);
	//ROS_INFO("RVs %d exist in the database", RVkeys.size() );
	for (int i = 0; i < RVkeys.size(); i++)
	{
	  //ROS_INFO("delete RTOV = %s", RVkeys.at(i).c_str());
	  _pdb->del(RVkeys.at(i));	  
	}
}

void delelteAllOCFromDB()
{
	vector <string> OCkeys = _pdb->getKeys(key::OC);
	//ROS_INFO("OCs %d exist in the database", OCkeys.size() );
	for (int i = 0; i < OCkeys.size(); i++)
	{
	  //ROS_INFO("delete OC = %s", OCkeys.at(i).c_str());
	  _pdb->del(OCkeys.at(i));
	}
	
}

void delelteAllSITOVFromDB()
{
	vector <string> SITOVkeys = _pdb->getKeys(key::SI);
	//ROS_INFO("SITOVs %d exist in the database", SITOVkeys.size() );
	for (int i = 0; i < SITOVkeys.size(); i++)
	{
	  //ROS_INFO("delete SITOV = %s", SITOVkeys.at(i).c_str());
	  //delete one TOVI from the db
	  _pdb->del(SITOVkeys.at(i));
	}
}

int deconceptualizingAllTrainData()
{
   delelteAllOCFromDB();
   delelteAllRVFromDB();
   delelteAllSITOVFromDB();
  
}
////////////////////////////////////////////////////////////////////////////////////////////////////

int crossValidationDataCRC(int K_fold, int iteration , string home_address)
{
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    ROS_INFO("\t\t[-]- Category Path = %s", package_path.c_str());
    string test_data_path = package_path + "/CV_test_instances.txt";
    // ROS_INFO("\t\t[-]- test Path = %s", test_data_path.c_str());
    std::ofstream testInstances (test_data_path.c_str(), std::ofstream::trunc);
    string train_data_path = package_path + "/CV_train_instances.txt";
    // ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
    
    std::ofstream trainInstances (train_data_path.c_str(), std::ofstream::trunc);
    
    // string path = home_address+ "/Category/Category.txt";
    string path = home_address + "Category/Category_orginal.txt";
    // string path = home_address+ "Category/Category.txt";
	ROS_INFO("\t\t[-]- database path = %s", path.c_str());

    std::ifstream listOfObjectCategories (path.c_str(), std::ifstream::in);

    int index =0;
    size_t total_number_of_instances = 0;

    while(listOfObjectCategories.good ())
    {
		string categoryAddress;
		std::getline (listOfObjectCategories, categoryAddress);
		if(categoryAddress.empty () || categoryAddress.at (0) == '#') // Skip blank lines or comments
		{
			continue;
		}
	
		// string cat_name = categoryAddress.c_str();
		// cat_name.resize(13);
		index ++;
		string category_address = home_address +"/"+ categoryAddress.c_str();
		std::ifstream categoryInstancesTmp (category_address.c_str());
		size_t category_size = 0;
		string pcd_file_address_tmp_tmp;
		string instance_name ;
		while (categoryInstancesTmp.good ())// read instances of a category 
		{
			std::getline (categoryInstancesTmp, pcd_file_address_tmp_tmp);
			if(pcd_file_address_tmp_tmp.empty () || pcd_file_address_tmp_tmp.at (0) == '#') // Skip blank lines or comments
			{
				continue;
			}
			instance_name = pcd_file_address_tmp_tmp;
			category_size++;
		}
		categoryInstancesTmp.close();
		string category_name = extractCategoryName(instance_name.c_str());
		ROS_INFO("\t\t[-]- Category %s has %ld object views ", category_name.c_str(), category_size);

		total_number_of_instances += category_size;
		// ROS_INFO("\t\t[-]- Category%i has %ld object views ", index, category_size);
	
		int test_index = int(category_size/K_fold) * (iteration);
		// ROS_INFO("\t\t[-]- test_index = %i", test_index);
		int i = 0;
		std::ifstream categoryInstances (category_address.c_str());
		
		while (categoryInstances.good ())// read instances of a category 
		{
			std::string pcd_file_address;
			std::getline (categoryInstances, pcd_file_address_tmp_tmp);
			if(pcd_file_address_tmp_tmp.empty () || pcd_file_address_tmp_tmp.at (0) == '#') // Skip blank lines or comments
			{
				continue;
			}
			
			if (iteration != K_fold-1) 
			{
				if ((i >= test_index) && (i < (int(category_size/K_fold) * (iteration + 1))))
				{
					testInstances << pcd_file_address_tmp_tmp<<"\n";
					// ROS_INFO("\t\t[%ld]- test data added", i);
				}
				else
				{
					if (category_name != "Unknown")
					{
						trainInstances << pcd_file_address_tmp_tmp<<"\n";
						// ROS_INFO("\t\t[%ld]- train data added", i );
					}    
					
				}
			}
			else 
			{
				if (i >= test_index) 
				{
					testInstances << pcd_file_address_tmp_tmp<<"\n";
					// ROS_INFO("\t\t[%ld]- test data added", i);
				}
				else
				{
					if (category_name != "Unknown")
					{
						trainInstances << pcd_file_address_tmp_tmp<<"\n";
						// ROS_INFO("\t\t[%ld]- train data added", i );
					}    	    
					
				}
			}
			i++;
		}
		categoryInstances.close();
    }
    testInstances.close();
    trainInstances.close();

    ROS_INFO("\t\t[-]- total_number_of_instances = %ld ", total_number_of_instances);

    return(1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////

int modelNetTrainTestData(string home_address)
{
	// #include <dirent.h>  //I use dirent.h for get folders in directory which is also available for windows:

    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string test_data_path = package_path + "/CV_test_instances.txt";
    std::ofstream testInstances (test_data_path.c_str(), std::ofstream::trunc);
    string train_data_path = package_path + "/CV_train_instances.txt";
    
    std::ofstream trainInstances (train_data_path.c_str(), std::ofstream::trunc);
    
 	DIR *d;
  	struct dirent *dir;  
	string dir_path = home_address + "Category/";
  	d = opendir(dir_path.c_str());
  
  	if (d)
  	{
		while ((dir = readdir(d)) != NULL)
		{
			string category_name = dir->d_name;
			//ROS_INFO("file/folder name = %s", strTmp.c_str());

			std::size_t found = category_name.find(".");
			if (found != std::string::npos)
			{
				continue;	
			}

			ROS_INFO("%s is a directory -- folder name should not contain dot [.]", category_name.c_str());
			
			/// train folder
			string dir_path_train = dir_path + category_name.c_str() + "/train/" ;
			ROS_INFO("train folder = %s", dir_path_train.c_str());
			DIR  *category;
			struct dirent *dir_tmp;  

			category = opendir(dir_path_train.c_str());

			if (category)
			{
				while ((dir_tmp = readdir(category)) != NULL)
				{
					string file_name = dir_tmp->d_name;
					std::size_t found_pcd = file_name.find(".pcd");
					std::size_t found_ply = file_name.find(".ply");

					if ((found_pcd != std::string::npos) || (found_ply != std::string::npos))
					{
						// ROS_INFO("%s is a pcd file ", strTmp.c_str());
						//ROS_INFO("file name = %s", file_name.c_str());
						trainInstances <<"Category//" << category_name.c_str() << "//train//" << file_name.c_str()<<"\n";
					}

				}
			}
	    	closedir(category);

			// Category//bathtub_Category//bathtub_object_1.pcd

			/// test folder
			string dir_path_test = dir_path + category_name.c_str() + "/test/" ;
			ROS_INFO("test folder = %s", dir_path_train.c_str());
			category = opendir(dir_path_test.c_str());

			if (category)
			{
				while ((dir_tmp = readdir(category)) != NULL)
				{
					string file_name = dir_tmp->d_name;
					std::size_t found_pcd = file_name.find(".pcd");
					std::size_t found_ply = file_name.find(".ply");

					if ((found_pcd != std::string::npos) || (found_ply != std::string::npos))					
					{
						// ROS_INFO("%s is a pcd file ", strTmp.c_str());
						// ROS_INFO("file name = %s", file_name.c_str());
						testInstances <<"Category//" << category_name.c_str() << "//test//" << file_name.c_str()<<"\n";
					}	
						
				}
			}
	    	closedir(category);		
		} 
	}

    closedir(d);
	testInstances.close();
	trainInstances.close();

    return(1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////
int objectRepresentationBagOfWords (vector <SITOV> cluster_center, 
				    vector <SITOV> object_spin_images, 
				    SITOV  &object_representation )
{
   
    if (object_spin_images.size() == 0 )
    {
		ROS_ERROR("Error: size of object view spin images is zero- could not represent based on BOW");
    }

    for (size_t i = 0; i < cluster_center.size(); i++)
    {
		object_representation.spin_image.push_back(0);
    }
    
    ROS_INFO("\t\t[-]- size of object view histogram = %ld", object_representation.spin_image.size());
    
    for (size_t i = 0; i < object_spin_images.size(); i++)
    {		
		SITOV sp1;
		sp1 = object_spin_images.at(i);
		
		float diffrence = 100;
		float diff_temp = 100;
		int id = 0;

		for (size_t j = 0; j < cluster_center.size(); j++)
		{
			SITOV sp2;
			sp2 = cluster_center.at(j);
			if (!differenceBetweenSpinImage(sp1, sp2, diffrence))
			{	
				ROS_INFO("\t\t[-]- size of spinimage of cluster center= %ld", sp2.spin_image.size());
				ROS_INFO("\t\t[-]- size of spinimage of the object= %ld" ,sp1.spin_image.size());
				ROS_ERROR("Error comparing spin images");
				return 0;	
			}
			//ROS_INFO("\t\t[-]- diffrence[%ld,%ld] = %f    diff_temp =%f",i ,j, diffrence, diff_temp);
			if ( diffrence < diff_temp)
			{
				diff_temp = diffrence;
				id = j;
			} 
		}
		//ROS_INFO("\t\t[-]- best_match =%i",id);
		object_representation.spin_image.at(id)++;
    }
    
    //normalizing histogram.
    for (size_t i = 0; i < cluster_center.size(); i++)
    {
		float normalizing_bin = object_representation.spin_image.at(i)/object_spin_images.size();
		object_representation.spin_image.at(i)= normalizing_bin;
  	} 
    return (1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
vector <int> generateSequence (int n)
{
    /* initialize random seed: */
    srand (time(NULL));
    vector <int> sequence;
    while( sequence.size() < n )
    {    
		/* generate random number between 1 and n: */
		int num = rand() % n + 1;
		bool falg = false;
		for (int j= 0; j < sequence.size(); j++)
		{
			if (num == sequence.at(j))
			{
				falg= true;
				break;
			}
		}
		if (falg == false)
		{
		sequence.push_back(num);
		}
		//ROS_INFO("\n size= %i \n",sequence.size()); 
    }
    
    return(sequence);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesInstances (string path, string home_address)
{    
    string path1= home_address + path;    
	// ROS_INFO("\n path-instance_original= %s \n",path1.c_str());

    std::ifstream listOfObjectInstancesAddress (path1.c_str());
    string InstanceAddresstmp = "";
    unsigned int number_of_exist_instances = 0;
    while (listOfObjectInstancesAddress.good()) 
    {
		std::getline (listOfObjectInstancesAddress, InstanceAddresstmp);
		if(InstanceAddresstmp.empty () || InstanceAddresstmp.at (0) == '#') // Skip blank lines or comments
			continue;
		number_of_exist_instances++;
    }
    // ROS_INFO("\n number_of_exist_instances= %i \n",number_of_exist_instances);
    
    InstanceAddresstmp = "";
    vector <int> instances_sequence = generateSequence (number_of_exist_instances);
    std::ofstream instances;
    string path2 = home_address + path;
    path2.resize(path2.size()-12);
    path2+= ".txt";
    //ROS_INFO("\n reorder category = %s \n",path2.c_str());

    instances.open (path2.c_str(), std::ofstream::out);
    for (int i =0; i < instances_sequence.size(); i++)
    {
		std::ifstream listOfObjectCategories (path1.c_str());
		int j = 0;
		while ((listOfObjectCategories.good()) && (j < instances_sequence.at(i)))
		{
			std::getline (listOfObjectCategories, InstanceAddresstmp);
			j++;
		}
		instances << InstanceAddresstmp.c_str()<<"\n";
		//ROS_INFO("\n instance= %s \n",InstanceAddresstmp.c_str());
    }
    instances.close();
    
    return (0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesCategoriesKfold ( string home_address , int number_of_object_per_category)
{
	ROS_INFO("generateRrandomSequencesCategories -- home_address = %s", home_address.c_str());

    std::string path;
    path = home_address +"/Category/Category_orginal.txt";
    std::ifstream listOfObjectCategoriesAddress (path.c_str());

    string categoryAddresstmp = "";
    unsigned int number_of_exist_categories = 0;
    while (listOfObjectCategoriesAddress.good()) 
    {
		std::getline (listOfObjectCategoriesAddress, categoryAddresstmp);
		if(categoryAddresstmp.empty () || categoryAddresstmp.at (0) == '#') // Skip blank lines or comments
			continue;
		number_of_exist_categories++;
    }
    
    vector <int> categories_sequence = generateSequence (number_of_exist_categories);
    std::ofstream categoies;
    string path2 = home_address +"/Category/Category.txt";
    categoies.open (path2.c_str(), std::ofstream::out);
    for (int i =0; i < categories_sequence.size(); i++)
    {
		std::ifstream listOfObjectCategories (path.c_str());
		int j = 0;
		while ((listOfObjectCategories.good()) && (j < categories_sequence.at(i)))
		{
			std::getline (listOfObjectCategories, categoryAddresstmp);
			generateRrandomSequencesInstances(categoryAddresstmp.c_str(), home_address);
			j++;
		}
		categoryAddresstmp.resize(categoryAddresstmp.size()-12);
		categoryAddresstmp+= ".txt";
		categoies << categoryAddresstmp.c_str()<<"\n";
    }
    return 0 ;
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesCategories ( string home_address , int number_of_object_per_category)
{
    std::string path;
    path = home_address +"/Category/Category_orginal.txt";
    std::ifstream listOfObjectCategoriesAddress (path.c_str());

    string categoryAddresstmp = "";
    unsigned int number_of_exist_categories = 0;
    while (listOfObjectCategoriesAddress.good()) 
    {
		std::getline (listOfObjectCategoriesAddress, categoryAddresstmp);
		if(categoryAddresstmp.empty () || categoryAddresstmp.at (0) == '#') // Skip blank lines or comments
			continue;
		number_of_exist_categories++;
    }
    
    vector <int> categories_sequence = generateSequence (number_of_exist_categories);
    std::ofstream categoies;
    string path2 = home_address +"/Category/Category.txt";
    categoies.open (path2.c_str(), std::ofstream::out);
    for (int i =0; i < categories_sequence.size(); i++)
    {
		std::ifstream listOfObjectCategories (path.c_str());
		int j = 0;
		while ((listOfObjectCategories.good()) && (j < categories_sequence.at(i)))
		{
			std::getline (listOfObjectCategories, categoryAddresstmp);
			generateRrandomSequencesInstances(categoryAddresstmp.c_str(), home_address);
			j++;
		}
		categoryAddresstmp.resize(categoryAddresstmp.size()-12);
		categoryAddresstmp+= ".txt";
		categoies << categoryAddresstmp.c_str()<<"\n";
    }
    return 0 ;
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void reportCurrentResults(int TP, int FP, int FN, string fname, bool global)
{

	FILE *result;
	double Precision = TP/double (TP+FP);
	double Recall = TP/double (TP+FN);
	float F1 = 2*(Precision*Recall)/(Precision+Recall);
	
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file.precision(4);
	if (global)
		Result_file << "\n\n\t******************* Global *********************";
	else	
		Result_file << "\n\n\t******************* Lastest run ****************";

	Result_file << "\n\t\t - True  Positive = "<< TP;
	Result_file << "\n\t\t - False Positive = "<< FP;
	Result_file << "\n\t\t - False Negative = "<< FN;
	Result_file << "\n\t\t - Precision  = "<< Precision;
	Result_file << "\n\t\t - Recall = "<< Recall;
	Result_file << "\n\t\t - F-measure = "<< F1;
	Result_file << "\n\n\t************************************************\n\n";
	
	Result_file << "\n------------------------------------------------------------------------------------------------------------------------------------";
	Result_file.close();
	ros::spinOnce();
} 
  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int addingGaussianNoise( boost::shared_ptr<PointCloud<PointT> > input_pc, 
						 double standard_deviation,
						 boost::shared_ptr<PointCloud<PointT> > output_pc)
{
	ROS_INFO ("Adding Gaussian noise with mean 0.0 and standard deviation %f\n", standard_deviation);
	*output_pc = *input_pc;

	boost::mt19937 rng; rng.seed (static_cast<unsigned int> (time (0)));
	boost::normal_distribution<> nd (0, standard_deviation);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

	for (size_t point_i = 0; point_i < input_pc->points.size (); ++point_i)
	{
		output_pc->points[point_i].x = input_pc->points[point_i].x + static_cast<float> (var_nor ());
		output_pc->points[point_i].y = input_pc->points[point_i].y + static_cast<float> (var_nor ());
		output_pc->points[point_i].z = input_pc->points[point_i].z + static_cast<float> (var_nor ());
	}

	return 0;
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int addingGaussianNoiseXYZL( boost::shared_ptr<PointCloud<pcl::PointXYZL> > input_pc, 
							 double standard_deviation,
							 boost::shared_ptr<PointCloud<pcl::PointXYZL> > output_pc)
{
	ROS_INFO ("Adding Gaussian noise with mean 0.0 and standard deviation %f\n", standard_deviation);
	*output_pc = *input_pc;

	boost::mt19937 rng; rng.seed (static_cast<unsigned int> (time (0)));
	boost::normal_distribution<> nd (0, standard_deviation);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

	for (size_t point_i = 0; point_i < input_pc->points.size (); ++point_i)
	{
		output_pc->points[point_i].x = input_pc->points[point_i].x + static_cast<float> (var_nor ());
		output_pc->points[point_i].y = input_pc->points[point_i].y + static_cast<float> (var_nor ());
		output_pc->points[point_i].z = input_pc->points[point_i].z + static_cast<float> (var_nor ());
	}

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int downSamplingXYZL( boost::shared_ptr<PointCloud<pcl::PointXYZL>  > cloud, 		
					  float downsampling_voxel_size, 
					  boost::shared_ptr<PointCloud<pcl::PointXYZL> > downsampled_pc)
{
	// Create the filtering object
  	VoxelGrid<PointXYZL > voxel_grid_downsampled_pc;
	voxel_grid_downsampled_pc.setInputCloud (cloud);
	voxel_grid_downsampled_pc.setLeafSize (downsampling_voxel_size, downsampling_voxel_size, downsampling_voxel_size);
	voxel_grid_downsampled_pc.filter (*downsampled_pc);
  	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int downSampling ( boost::shared_ptr<PointCloud<PointT> > cloud, 		
					double downsampling_voxel_size, 
					boost::shared_ptr<PointCloud<PointT> > downsampled_pc)
{
	// Create the filtering object
  	VoxelGrid<PointT> voxel_grid_downsampled_pc;
	voxel_grid_downsampled_pc.setInputCloud (cloud);
	voxel_grid_downsampled_pc.setLeafSize (downsampling_voxel_size, downsampling_voxel_size, downsampling_voxel_size);
	voxel_grid_downsampled_pc.filter (*downsampled_pc);

  	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int estimateViewpointFeatureHistogram(boost::shared_ptr<PointCloud<PointT> > cloud, 
				    float normal_estimation_radius,
				    pcl::PointCloud<pcl::VFHSignature308>::Ptr &vfhs)
{ 
	// 	const size_t VFH_size = 308;	
	// 	ROS_INFO("VFH_size = %ld",VFH_size);

	// 	//STEP 1: Downsample the input point cloud using downsampling voxel size
	// 	PointCloud<PointT>::Ptr downsampled_pc (new PointCloud<PointT>);
	// 	PointCloud<int> sampled_indices;
	// 	UniformSampling<PointT> uniform_sampling;
	// 	uniform_sampling.setInputCloud (cloud);
	// 	uniform_sampling.setRadiusSearch (downsampling_voxel_size/*0.01f*/);
	// 	uniform_sampling.compute (sampled_indices);//indices means "fehrest";
	// 	copyPointCloud (*cloud, sampled_indices.points, *downsampled_pc);//Keypoints = voxel grid downsampling 
	// 	
	//STEP 2: Compute normals for downsampled point cloud
	search::KdTree<PointT>::Ptr kdtree (new search::KdTree<PointT>);
	NormalEstimation<PointT, Normal> normal_estimation;
	normal_estimation.setInputCloud (cloud);
	normal_estimation.setSearchMethod (kdtree);
	normal_estimation.setRadiusSearch ( normal_estimation_radius/*0.05*/);
	PointCloud<Normal>::Ptr normal (new PointCloud< Normal>);
	normal_estimation.compute (*normal);

	//STEP 3: Estimate the VFH for the downsampled_pc with the downsampled_pc_with_normals
	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (normal);
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	vfh.setSearchMethod (tree);
	// pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
	vfh.compute (*vfhs);

	// ROS_INFO("****VFH_size = %ld",vfhs->size ());
	
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingVFHTrainData( int &track_id, 
				PrettyPrint &pp,
				string home_address, 
				float normal_estimation_radius)
{  
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
    //int track_id =1;
    
    while (train_data.good ())// read train address
    {	
		std::getline (train_data, pcd_file_address_tmp);
		if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
		{
			continue;
		}

		string pcd_file_address= home_address + pcd_file_address_tmp;
		pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
		//load a PCD object   
		boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
		try 
		{ 
			io::loadPCDFile <PointXYZRGBA> (pcd_file_address.c_str(), *target_pc) ;
		} 
		catch (const std::exception& e) 
		{ 
			ROS_ERROR("\t\t[-] could not read given object %s :", pcd_file_address.c_str());					
			return(0);
		}
		pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );
		
		/* __________________________________________________________
		|                                                           |
		|  Compute the new shape description for given point cloud  |
		|___________________________________________________________| */

		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh (new pcl::PointCloud<pcl::VFHSignature308> ());
		// boost::shared_ptr< vector <SITOV> > vfh (new vector <SITOV>);
		estimateViewpointFeatureHistogram(  target_pc, 
											normal_estimation_radius,
											vfh);

		size_t vfh_size = sizeof(vfh->points.at(0).histogram)/sizeof(float);
		
		SITOV object_representation;
		for (size_t i = 0; i < vfh_size ; i++)
		{
			object_representation.spin_image.push_back( vfh->points.at(0).histogram[i]);
		}
		//ROS_INFO("VFH_size = %ld",tmp.spin_image.size());

		object_representation.ground_truth_name = pcd_file_address.c_str();

		std::string categoryName;
		categoryName = extractCategoryName(pcd_file_address_tmp);

		addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
		track_id++;
    }
    return (1);
 }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingVFHDownSampledTrainData( int &track_id, 
				PrettyPrint &pp,
				string home_address, 
				float normal_estimation_radius,
				float downsampling_voxel_size)
{
   
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
    //int track_id =1;
    
    while (train_data.good ())// read train address
    {	
		std::getline (train_data, pcd_file_address_tmp);
		if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
		{
			continue;
		}

		string pcd_file_address= home_address + pcd_file_address_tmp;
		pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
		//load a PCD object   
		boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
		if (pcl::io::loadPCDFile <PointXYZRGBA> (pcd_file_address.c_str(), *target_pc) == -1)
		{	
			ROS_ERROR("\t\t[-]-Could not read given object %s :",pcd_file_address.c_str());
			return(0);
		}
		pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );
		
		//downSampling
		downSampling (  target_pc, 		
						downsampling_voxel_size,
						target_pc);
		
		/* __________________________________________________________
		|                                                           |
		|  Compute the new shape description for given point cloud  |
		|___________________________________________________________| */

		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh (new pcl::PointCloud<pcl::VFHSignature308> ());
		// boost::shared_ptr< vector <SITOV> > vfh (new vector <SITOV>);
		estimateViewpointFeatureHistogram(target_pc, 
						normal_estimation_radius,
						vfh);

		size_t vfh_size = sizeof(vfh->points.at(0).histogram)/sizeof(float);
		
		SITOV object_representation;
		for (size_t i = 0; i < vfh_size ; i++)
		{
			object_representation.spin_image.push_back( vfh->points.at(0).histogram[i]);
		}
		//ROS_INFO("VFH_size = %ld",tmp.spin_image.size());
			
		std::string categoryName;
		categoryName = extractCategoryName(pcd_file_address_tmp);

		addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
		track_id++;
    }
    return (1);
 }

//////////////////////////////////////////////////////////////////////////////////////
int conceptualizingESFTrainData( int &track_id, 
								 PrettyPrint &pp,
								 string home_address)
{
   
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    // ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
   
    while (train_data.good ())// read train address
    {	
		std::getline (train_data, pcd_file_address_tmp);
		if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
		{
			continue;
		}

		string pcd_file_address = home_address + pcd_file_address_tmp;
		//pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
		//load a PCD object   
		boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
		try 
		{ 
			io::loadPCDFile <PointXYZRGBA> (pcd_file_address.c_str(), *target_pc) ;
		} 
		catch (const std::exception& e) 
		{ 
			ROS_ERROR("\t\t[-] could not read given object %s :", pcd_file_address.c_str());					
			return(0);
		}
		
		//pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );
		
		/* __________________________________________________________
		|                                                           |
		|  Compute the ESF shape description for given point cloud  |
		|___________________________________________________________| */

		pcl::PointCloud<pcl::ESFSignature640>::Ptr esf (new pcl::PointCloud<pcl::ESFSignature640> ());
		estimateESFDescription (target_pc, esf);
		size_t esf_size = sizeof(esf->points.at(0).histogram)/sizeof(float);
		SITOV object_representation;
		for (size_t i = 0; i < esf_size ; i++)
		{
			object_representation.spin_image.push_back( esf->points.at(0).histogram[i]);
		}
		// ROS_INFO("ESF_size = %ld",object_representation.spin_image.size());

		object_representation.ground_truth_name = pcd_file_address.c_str();

			
		std::string categoryName;
		categoryName = extractCategoryName(pcd_file_address_tmp);

		addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
		track_id++;
    }
    return (1);
 }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingESFDownSampledTrainData( int &track_id, 
											PrettyPrint &pp,
											string home_address,
											float downsampling_voxel_size)
{
   
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
    //int track_id =1;
    
    while (train_data.good ())// read train address
    {	
	std::getline (train_data, pcd_file_address_tmp);
	if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
	{
	    continue;
	}

	string pcd_file_address= home_address + pcd_file_address_tmp;
	pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
	//load a PCD object   
	boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
	if (pcl::io::loadPCDFile <PointXYZRGBA> (pcd_file_address.c_str(), *target_pc) == -1)
	{	
	    ROS_ERROR("\t\t[-]-Could not read given object %s :",pcd_file_address.c_str());
	    return(0);
	}
	pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );

	//downSampling
	downSampling (  target_pc, 		
			downsampling_voxel_size,
			target_pc);

	/* __________________________________________________________
	|                                                           |
	|  Compute the ESF shape description for given point cloud  |
	|___________________________________________________________| */


	pcl::PointCloud<pcl::ESFSignature640>::Ptr esf (new pcl::PointCloud<pcl::ESFSignature640> ());
	estimateESFDescription (target_pc, esf);
	size_t esf_size = sizeof(esf->points.at(0).histogram)/sizeof(float);
	SITOV object_representation;
	for (size_t i = 0; i < esf_size ; i++)
	{
	    object_representation.spin_image.push_back( esf->points.at(0).histogram[i]);
	}
	//ROS_INFO("ESF_size = %ld",object_representation.spin_image.size());

		
	std::string categoryName;
	categoryName = extractCategoryName(pcd_file_address_tmp);

	addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
	track_id++;
    }
    return (1);
 }
 
////////////////////////////////////////////////////////////////////////////////////// 
int estimateGRSDDescription(boost::shared_ptr<PointCloud<PointT> > cloud, 
							float normal_estimation_radius,
							pcl::PointCloud<pcl::GRSDSignature21>::Ptr &grsds)
{

	// Estimate the normals.
	search::KdTree<PointT>::Ptr kdtree (new search::KdTree<PointT>);
	NormalEstimation<PointT, Normal> normal_estimation;
	normal_estimation.setInputCloud (cloud);
	normal_estimation.setSearchMethod (kdtree);
	normal_estimation.setRadiusSearch ( normal_estimation_radius);
	PointCloud<Normal>::Ptr normal (new PointCloud< Normal>);
	normal_estimation.compute (*normal);	
  
	// GRSD estimation object.
	pcl::GRSDEstimation<PointT, pcl::Normal, pcl::GRSDSignature21> grsd;
	grsd.setInputCloud(cloud);
	grsd.setInputNormals(normal);
	grsd.setSearchMethod(kdtree);
	// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
	grsd.setRadiusSearch(0.05);
	grsd.compute(*grsds);
	
	return 0;
} 
   
//////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGRSDTrainData(   int &track_id, 
									PrettyPrint &pp,		  
									string home_address,
									float normal_estimation_radius )
{
   
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
    //int track_id =1;
    
    while (train_data.good ())// read train address
    {	
	std::getline (train_data, pcd_file_address_tmp);
	if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
	{
	    continue;
	}

	string pcd_file_address= home_address + pcd_file_address_tmp;
	pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
	//load a PCD object   
	boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
	try 
	{ 
		io::loadPCDFile <PointXYZRGBA> (pcd_file_address.c_str(), *target_pc) ;
	} 
	catch (const std::exception& e) 
	{ 
		ROS_ERROR("\t\t[-] could not read given object %s :", pcd_file_address.c_str());					
		return(0); 
	}
	pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );


	/* __________________________________________________________
	|                                                            |
	|  Compute the GRSD shape description for given point cloud  |
	|____________________________________________________________| */

	pcl::PointCloud<pcl::GRSDSignature21>::Ptr grsd (new pcl::PointCloud<pcl::GRSDSignature21> ());
	estimateGRSDDescription(target_pc, normal_estimation_radius, grsd);
	
	size_t grsd_size = sizeof(grsd->points.at(0).histogram)/sizeof(float);
	SITOV object_representation;
	for (size_t i = 0; i < grsd_size ; i++)
	{
	    object_representation.spin_image.push_back( grsd->points.at(0).histogram[i]);
	}
	//ROS_INFO("GRSD_size = %ld",object_representation.spin_image.size());
	
	object_representation.ground_truth_name = pcd_file_address.c_str();
		
	std::string category_name;
	category_name = extractCategoryName(pcd_file_address_tmp);
	addObjectViewHistogramInSpecificCategory(category_name, 1, track_id, 1, object_representation , pp);	
	track_id++;
    }
    return (1);
 }
 
 //////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGRSDDownSampledTrainData( int &track_id, 
											 PrettyPrint &pp,
											 string home_address,
											 float normal_estimation_radius,
											 float downsampling_voxel_size)
{
   
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
    //int track_id =1;
    
    while (train_data.good ())// read train address
    {	
	std::getline (train_data, pcd_file_address_tmp);
	if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
	{
	    continue;
	}

	string pcd_file_address= home_address + pcd_file_address_tmp;
	pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
	//load a PCD object   
	boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
	if (pcl::io::loadPCDFile <PointXYZRGBA> (pcd_file_address.c_str(), *target_pc) == -1)
	{	
	    ROS_ERROR("\t\t[-]-Could not read given object %s :",pcd_file_address.c_str());
	    return(0);
	}
	pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );

	//downSampling
	downSampling( target_pc, 		
				  downsampling_voxel_size,
				  target_pc);

	/* __________________________________________________________
	|                                                            |
	|  Compute the GRSD shape description for given point cloud  |
	|____________________________________________________________| */

	pcl::PointCloud<pcl::GRSDSignature21>::Ptr grsd (new pcl::PointCloud<pcl::GRSDSignature21> ());
	estimateGRSDDescription(target_pc, normal_estimation_radius, grsd);
	
	size_t grsd_size = sizeof(grsd->points.at(0).histogram)/sizeof(float);
	SITOV object_representation;
	for (size_t i = 0; i < grsd_size ; i++)
	{
	    object_representation.spin_image.push_back( grsd->points.at(0).histogram[i]);
	}
	//ROS_INFO("GRSD_size = %ld",object_representation.spin_image.size());

		
	std::string categoryName;
	categoryName = extractCategoryName(pcd_file_address_tmp);

	addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
	track_id++;
    }
    return (1);
 }
  
////////////////////////////////////////////////////////////////////////////////////// 
int estimateGFPFH(boost::shared_ptr<PointCloud<pcl::PointXYZL> > cloud, 
		   			pcl::PointCloud<pcl::GFPFHSignature16>::Ptr &gfpfhs)
{

	// Note: you should now perform classification on the cloud's points. See the original paper for more details. 
	//For this example, we will now consider 16 different classes, and randomly label each point as one of them.
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].label = 1 + i % 16;
	}
 
	// GFPFH estimation object.
	pcl::GFPFHEstimation<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16> gfpfh;
	gfpfh.setInputCloud(cloud);
	// Set the object that contains the labels for each point. Thanks to the
	// PointXYZL type, we can use the same object we store the cloud in.
	gfpfh.setInputLabels(cloud);
	// Set the size of the octree leaves to 1cm (cubic).
	gfpfh.setOctreeLeafSize(0.01);
	// Set the number of classes the cloud has been labelled with (default is 16).
	gfpfh.setNumberOfClasses(16);
	gfpfh.compute(*gfpfhs);
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGFPFHTrainData( int &track_id, 
									PrettyPrint &pp,
									string home_address)
{
   
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
    //int track_id =1;
    
    while (train_data.good ())// read train address
    {	
	std::getline (train_data, pcd_file_address_tmp);
	if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
	{
	    continue;
	}

	string pcd_file_address= home_address + pcd_file_address_tmp;
	pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
	//load a PCD object     
	boost::shared_ptr<PointCloud<pcl::PointXYZL> > target_pc (new PointCloud<pcl::PointXYZL>);
	if (pcl::io::loadPCDFile <pcl::PointXYZL> (pcd_file_address.c_str(), *target_pc) == -1)
	{	
		ROS_ERROR("\t\t[-]-Could not read given object %s :",pcd_file_address.c_str());
		return(0);
	}
	pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );
	
	/* ___________________________________________________________
	|                                                             |
	|  Compute the GFPFH shape description for given point cloud  |
	|_____________________________________________________________| */

	pcl::PointCloud<pcl::GFPFHSignature16>::Ptr gfpfh (new pcl::PointCloud<pcl::GFPFHSignature16> ());
	estimateGFPFH( target_pc, gfpfh);
	
	size_t gfpfh_size = sizeof(gfpfh->points.at(0).histogram)/sizeof(float);
	SITOV object_representation;
	for (size_t i = 0; i < gfpfh_size ; i++)
	{
	    object_representation.spin_image.push_back( gfpfh->points.at(0).histogram[i]);
	}		
	//ROS_INFO("GFPFH_size = %ld",object_representation.spin_image.size());

	std::string categoryName;
	categoryName = extractCategoryName(pcd_file_address_tmp);

	addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
	track_id++;
    }
    return (1);
 }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGFPFHDownSampledTrainData( int &track_id, 
											  PrettyPrint &pp,
											  string home_address, 
											  float downsampling_voxel_size )
{
   
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
    //int track_id =1;
    
    while (train_data.good ())// read train address
    {	
		std::getline (train_data, pcd_file_address_tmp);
		if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
		{
			continue;
		}

		string pcd_file_address= home_address + pcd_file_address_tmp;
		pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
		
		boost::shared_ptr<PointCloud<pcl::PointXYZL> > target_pc (new PointCloud<pcl::PointXYZL>);
		if (pcl::io::loadPCDFile <pcl::PointXYZL> (pcd_file_address.c_str(), *target_pc) == -1)
		{	
			ROS_ERROR("\t\t[-]-Could not read given object %s :",pcd_file_address.c_str());
			return(0);
		}
		pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );
		
		downSamplingXYZL( target_pc, 		
						  downsampling_voxel_size,
						  target_pc);
				
		/* ___________________________________________________________
		|                                                             |
		|  Compute the GFPFH shape description for given point cloud  |
		|_____________________________________________________________| */

		pcl::PointCloud<pcl::GFPFHSignature16>::Ptr gfpfh (new pcl::PointCloud<pcl::GFPFHSignature16> ());
		estimateGFPFH( target_pc, gfpfh);
		
		size_t gfpfh_size = sizeof(gfpfh->points.at(0).histogram)/sizeof(float);
		SITOV object_representation;
		for (size_t i = 0; i < gfpfh_size ; i++)
		{
			object_representation.spin_image.push_back( gfpfh->points.at(0).histogram[i]);
		}		
		//ROS_INFO("GFPFH_size = %ld",object_representation.spin_image.size());

		std::string categoryName;
		categoryName = extractCategoryName(pcd_file_address_tmp);

		addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
		track_id++;
    }
    return (1);
 }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingDeepLearningBasedOnRGBAndOrthographicImages( int &track_id, 
																string dataset_path, 																
																ros::ServiceClient deep_learning_server, 
																int number_of_bins,																
																PrettyPrint &pp)
{  
    int adaptive_support_lenght=1;
	double global_image_width=0.5;
	int threshold=1000;
	bool downsampling=false; 
	double downsampling_voxel_size=1000;

    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-] train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp;
    //int track_id =1;
    //ofstream train_csv;
	//train_csv.open ("/home/hamidreza/Desktop/train.csv", std::ofstream::trunc);

    while (train_data.good ())// read train address
    {	
		std::getline (train_data, pcd_file_address_tmp);
		if(pcd_file_address_tmp.empty () || pcd_file_address_tmp.at (0) == '#') // Skip blank lines or comments
		{
			continue;
		}

		ROS_INFO("\t\t[-] object = %s", pcd_file_address_tmp.c_str());
		string pcd_file_address= dataset_path + pcd_file_address_tmp;
		pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
		//load a PCD object   
		boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
		// if (pcl::io::loadPCDFile <PointT> (pcd_file_address.c_str(), *target_pc) == -1)
		// {	
		// 	ROS_ERROR("\t\t[-] could not read given object %s :",pcd_file_address.c_str());
		// 	continue;
		// }
		try 
		{ 
			io::loadPCDFile <PointXYZRGBA> (pcd_file_address.c_str(), *target_pc) ;
		} 
		catch (const std::exception& e) 
		{ 
			ROS_ERROR("\t\t[-] could not read given object %s :", pcd_file_address.c_str());					
			continue; 
		}
		
		//pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );
	
		//downsampling 0 = false 1 = true
		if (downsampling)
        {
			ROS_INFO("\t\t[-] size of object before downsampling (%f) = %d",downsampling_voxel_size, target_pc->points.size());

            downSampling ( target_pc, 		
			               downsampling_voxel_size,
                           target_pc); 

            ROS_INFO("\t\t[-] size of object after downsampling (%f) = %d",downsampling_voxel_size, target_pc->points.size());   
        }

		/* __________________________________________________________
		|                                                           |
		|  Compute the new shape description for given point cloud  |
		|___________________________________________________________| */

		boost::shared_ptr<pcl::PointCloud<T> > pca_object_view (new PointCloud<PointT>);
		boost::shared_ptr<PointCloud<PointT> > pca_pc (new PointCloud<PointT>); 
		vector < boost::shared_ptr<pcl::PointCloud<PointT> > > vector_of_projected_views;
		double largest_side = 0;
		int  sign = 1;
		vector <float> view_point_entropy;
		string list_of_sorted_projected_plane;
		Eigen::Vector3f center_of_bbox (0.0, 0.0, 0.0);
        vector< float > object_description;

		computeDepthBasedGoodDescriptorAndSaveDepthAndRGBProjections( target_pc,
																	adaptive_support_lenght,
																	global_image_width,
																	threshold,
																	number_of_bins,
																	pca_object_view,
																	center_of_bbox,
																	vector_of_projected_views, 
																	largest_side, 
																	sign,
																	view_point_entropy,
																	list_of_sorted_projected_plane,
																	object_description );
		
		string view_with_max_enntropy = list_of_sorted_projected_plane;
		view_with_max_enntropy.resize(3);													
		ROS_INFO ("\t\t[-] list_of_sorted_projected_plane =  %s", list_of_sorted_projected_plane.c_str());
		ROS_INFO ("\t\t[-] view_with_max_enntropy = %s", view_with_max_enntropy.c_str());

		SITOV object_representation;
		for (size_t i = 0; i < object_description.size(); i++)
		{
			object_representation.spin_image.push_back(object_description.at(i));
		}

		ROS_INFO("\t\t[-] size of object view histogram %ld",object_representation.spin_image.size());
		
		
 		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		cv_ptr->encoding = "bgr8";
		string img_name = "/tmp/"+ view_with_max_enntropy +"_RGB.jpg";
		ROS_INFO ("\t\t[-] view_with_max_entropy_path = %s", img_name.c_str());
		cv_ptr->image = cv::imread(img_name.c_str(), CV_LOAD_IMAGE_COLOR);      

		SITOV deep_representation_sitov;
		/// call deep learning service to represent the given GOOD description as vgg16  
		rug_deep_feature_extraction::deep_representation srv;
		srv.request.good_representation = object_representation.spin_image;
		srv.request.RGB_image = *cv_ptr->toImageMsg();

		cv_bridge::CvImagePtr cv_ptr_depth(new cv_bridge::CvImage);
		cv_ptr_depth->encoding = "bgr8";

		img_name = "/tmp/"+ view_with_max_enntropy +"_merged.jpg";
		cv_ptr_depth->image = cv::imread(img_name.c_str(), CV_LOAD_IMAGE_COLOR);      
		srv.request.depth_image = *cv_ptr_depth->toImageMsg();

		if (deep_learning_server.call(srv))
		{
			//pp.info(std::ostringstream().flush() << "################ receive server responce with size of " << srv.response.deep_representation.size() );
			ROS_INFO("\t\t[-] size of deep object representation = %ld", srv.response.deep_representation.size() );
			if (srv.response.deep_representation.size() < 1)
				ROS_ERROR("Failed to call service deep learning service");
				
			for (size_t i = 0; i < srv.response.deep_representation.size(); i++)
			{
				deep_representation_sitov.spin_image.push_back(srv.response.deep_representation.at(i));
				//train_csv << srv.response.deep_representation.at(i) << ",";
			}
		}
		else
		{
			ROS_ERROR("Failed to call deep learning service");
			continue;
		}

		deep_representation_sitov.ground_truth_name = pcd_file_address.c_str();
		
		// char ch;
        // ch = getchar ();

		std::string categoryName;
		categoryName = extractCategoryName(pcd_file_address_tmp);
		//train_csv << categoryName << ",";
		//train_csv << "\n";
		
		ros::Time start_time = ros::Time::now();
	   	addObjectViewHistogramInSpecificCategoryDeepLearning(categoryName, 1, track_id, 1, deep_representation_sitov , pp);	
		//ROS_INFO("**** add a new object to PDB took = %f", (ros::Time::now() - start_time).toSec());
		ROS_INFO("\t\t[-] =========================\n");
		track_id++;
    }
	//train_csv.close();
    return (1);
 }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int conceptualizingGOODTrainData( int &track_id, 
								  PrettyPrint &pp,
								  string home_address, 
								  int adaptive_support_lenght,
								  double global_image_width,
								  int threshold,
								  int number_of_bins)
{
   
    string package_path  = ros::package::getPath("rug_kfold_cross_validation");
    string train_data_path = package_path + "/CV_train_instances.txt";
    std::ifstream train_data (train_data_path.c_str(), std::ifstream::in);
    ROS_INFO("\t\t[-]- train Path = %s", train_data_path.c_str());
 	
    string pcd_file_address_tmp_tmp;
    //int track_id =1;
    
    while (train_data.good ())// read train address
    {	
		std::getline (train_data, pcd_file_address_tmp_tmp);
		if(pcd_file_address_tmp_tmp.empty () || pcd_file_address_tmp_tmp.at (0) == '#') // Skip blank lines or comments
		{
			continue;
		}

		string pcd_file_address= home_address + pcd_file_address_tmp_tmp;
		pp.info(std::ostringstream().flush() << "path: " << pcd_file_address.c_str());
		//load a PCD object   
		boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
		try 
		{ 
			io::loadPCDFile <PointXYZRGBA> (pcd_file_address.c_str(), *target_pc) ;
		} 
		catch (const std::exception& e) 
		{ 
			ROS_ERROR("\t\t[-] could not read given object %s :", pcd_file_address.c_str());					
			return(0);
		}

		pp.info(std::ostringstream().flush() << "The size of given point cloud  = " << target_pc->points.size() );
		
		/* __________________________________________________________
		|                                                           |
		|  Compute the new shape description for given point cloud  |
		|___________________________________________________________| */

		boost::shared_ptr<pcl::PointCloud<T> > pca_object_view (new PointCloud<PointT>);
		boost::shared_ptr<PointCloud<PointT> > pca_pc (new PointCloud<PointT>); 
		vector < boost::shared_ptr<pcl::PointCloud<PointT> > > vector_of_projected_views;
		double largest_side = 0;
		int  sign = 1;
		vector <float> view_point_entropy;
		string std_name_of_sorted_projected_plane;

		Eigen::Vector3f center_of_bbox;

		vector< float > object_description;

		compuet_object_description( target_pc,
									adaptive_support_lenght,
									global_image_width,
									threshold,
									number_of_bins,
									pca_object_view,
									center_of_bbox,
									vector_of_projected_views, 
									largest_side, 
									sign,
									view_point_entropy,
									std_name_of_sorted_projected_plane,
									object_description );
		
		SITOV object_representation;
		for (size_t i = 0; i < object_description.size(); i++)
		{
			object_representation.spin_image.push_back(object_description.at(i));
		}
		//ROS_INFO("\nsize of object view histogram %ld",object_representation.spin_image.size());
		
		object_representation.ground_truth_name = pcd_file_address.c_str();


		std::string categoryName;
		categoryName = extractCategoryName(pcd_file_address_tmp_tmp);

		addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
		track_id++;
    }
    return (1);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool fexists(std::string filename) 
{
  ifstream ifile(filename.c_str());
  return ifile.good();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int numberOfPerformedExperiments(string name_of_approach, int &exp_num)
{
    int number_of_exist_experiments = 0;
    std::string resultsOfExperiments;
    resultsOfExperiments = ros::package::getPath("rug_kfold_cross_validation")+ "/result/results_of_"+name_of_approach+"_experiments.txt";
    ROS_INFO("results of expriments file path = %s",resultsOfExperiments.c_str() );

    if (!fexists(resultsOfExperiments.c_str()))
    {
		ROS_INFO("File not exist");
		exp_num=1;
    }
    else
    {
		ROS_INFO("File exist");
		string tmp;
		std::ifstream num_results_of_experiments;
		num_results_of_experiments.open (resultsOfExperiments.c_str());
		while (num_results_of_experiments.good()) 
		{
			std::getline (num_results_of_experiments, tmp);
			if(tmp.empty () || tmp.at (0) == '#') // Skip blank lines or comments
			continue;
			number_of_exist_experiments++;
		}
		num_results_of_experiments.close();
		exp_num = (number_of_exist_experiments)/2;
		ROS_INFO("number_of_exist_experiments = %i",exp_num );
    }
        
    return 0;    
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int reportAllDeepTransferLearningExperiments (  int TP, int FP, int FN,
											    string dataset,
											    int orthographic_image_resolution, 
												string deep_learning_architecture,											
												string pooling_function,
												string distance_function, 
												int K,
												double avg_class_accuracy, 
												string name_of_approach, 
												double duration_sec)
{
    //ROS_INFO("TEST report_all_experiments_results fucntion");   
    int number_of_exist_experiments = 0;
    double precision = TP / double (TP+FP);
    double recall = TP / double (TP+FN);
    
    double f_measure;
    if ((precision + recall)==0)
		f_measure = 10000;//means infinite
    else
		f_measure = ( 2 * precision * recall) / double(precision + recall);
	
    std::string log_file_path;
    log_file_path = ros::package::getPath("rug_kfold_cross_validation")+ "/result/results_of_"+name_of_approach+"_experiments.txt";
    ROS_INFO("results of expriments file path = %s",log_file_path.c_str() );
    int exp_num =1;

    if (!fexists(log_file_path.c_str()))
    {
		ROS_INFO("File not exist");
		std::ofstream results_of_experiments;
		results_of_experiments.open (log_file_path.c_str(), std::ofstream::out);
		results_of_experiments.precision(4);
		results_of_experiments << "EXP"<<"\tnetwork" <<"\t\tdataset" <<"\t\tresolution" << "\t"<< "pooling" << "\t\t"<< "dist_func" << "\t\tK\t\t"<<  "Ins-Acc"<< "\t\t"<< "Avg-Class-Acc"<< "\t"<< "time (s)";
		results_of_experiments << "\n------------------------------------------------------------------------------------------------------------------------------------------------------------------------------";
		results_of_experiments << "\n"<<exp_num << "\t"<< deep_learning_architecture << "\t"<< dataset << "\t" << orthographic_image_resolution  <<"\t\t"<< pooling_function <<"\t\t"<< distance_function <<"\t\t"<< K << "\t\t"<< f_measure << "\t\t"<< avg_class_accuracy<< "\t\t"<< duration_sec;
		results_of_experiments << "\n------------------------------------------------------------------------------------------------------------------------------------------------------------------------------";
		results_of_experiments.close();
		results_of_experiments.clear();
    }
    else
    {
		ROS_INFO("File exist");
		string tmp;
		std::ifstream num_results_of_experiments;
		num_results_of_experiments.open (log_file_path.c_str());
		while (num_results_of_experiments.good()) 
		{
			std::getline (num_results_of_experiments, tmp);
			if(tmp.empty () || tmp.at (0) == '#') // Skip blank lines or comments
			continue;
			number_of_exist_experiments++;
		}
		num_results_of_experiments.close();
		exp_num = (number_of_exist_experiments)/2;
		ROS_INFO("number_of_exist_experiments = %i",exp_num );

		std::ofstream results_of_experiments;
		results_of_experiments.open (log_file_path.c_str(), std::ofstream::app);
		results_of_experiments.precision(4);
		results_of_experiments << "\n"<<exp_num << "\t"<< deep_learning_architecture << "\t"<< dataset << "\t" << orthographic_image_resolution  <<"\t\t"<< pooling_function <<"\t\t"<< distance_function <<"\t\t"<< K << "\t\t"<< f_measure << "\t\t"<< avg_class_accuracy<< "\t\t"<< duration_sec;
		results_of_experiments << "\n------------------------------------------------------------------------------------------------------------------------------------------------------------------------------";
		results_of_experiments.close();
		results_of_experiments.clear();
		
    } 
   
	return 0;	
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int reportAllHandCraftedExperiments( 	int TP, int FP, int FN, 
										double avg_class_accuracy, 
										string object_descriptor, 
										double descriptor_param, 
										int K,
										string distance_function, 										 
										double duration_sec, string name_of_approach)
{
    //ROS_INFO("TEST reportAllExperiments fucntion");   
    int number_of_exist_experiments = 0;
    double precision = TP/double (TP+FP);
    double recall = TP/double (TP+FN);
    
    double f_measure;
    if ((precision + recall) == 0)
		f_measure = 10000; //means infinite
    else
		f_measure = (2 * precision * recall) / double(precision + recall);

    std::string log_file_path;
    log_file_path = ros::package::getPath("rug_kfold_cross_validation")+ "/result/results_of_"+name_of_approach+"_experiments.txt";
    ROS_INFO("results of expriments file path = %s",log_file_path.c_str() );
    int exp_num = 1;

    if (!fexists(log_file_path.c_str()))
    {
		ROS_INFO("File not exist");
		std::ofstream results_of_experiments;
		results_of_experiments.open (log_file_path.c_str(), std::ofstream::out);
		results_of_experiments.precision(4);
		if (object_descriptor == "GOOD")
		{
			results_of_experiments << "EXP"<< "\t" << "Obj.Disc." << "\t"<< "#bins\t"<< "K"<< "\tDist.Func."<< "\t\t"<< "Ins-Acc" << "\t"<< "Avg-Class-Acc"<< "\tTime(s)";
			results_of_experiments << "\n---------------------------------------------------------------------------------------------------------";       
			results_of_experiments << "\n" << exp_num << "\t" << object_descriptor <<"\t\t"<< descriptor_param <<"\t" << K <<"\t"<< distance_function << "\t\t"<< f_measure << "\t"<< avg_class_accuracy << "\t\t"<< duration_sec;        		    
		
		}	
		else if (object_descriptor == "VFH")
		{
			results_of_experiments << "EXP"<< "\t" << "Descriptor." << "\t"<< "radius"<< "\t\t"<< "Dist.Func."<< "\t\t"<< "Ins-Acc" << "\t"<< "Avg-Class-Acc"<<"\tTime(s)";
			results_of_experiments << "\n---------------------------------------------------------------------------------------------------------";       
			results_of_experiments << "\n" << exp_num << "\t" << object_descriptor <<"\t\t"<< descriptor_param <<"\t" << K <<"\t"<< distance_function << "\t\t"<< f_measure << "\t"<< avg_class_accuracy << "\t\t"<< duration_sec;
		
		}
		else if (object_descriptor == "ESF")
		{
			results_of_experiments << "EXP"<< "\t" << "Descriptor." << "\t\t"<< "Dist.Func."<< "\t\t"<< "Ins-Acc" << "\t"<< "Avg-Class-Acc"<<"\tTime(s)";
			results_of_experiments << "\n---------------------------------------------------------------------------------------------------------";       
			results_of_experiments << "\n" << exp_num << "\t" << object_descriptor <<"\t\t"<< "---" <<"\t" << K <<"\t"<< distance_function << "\t\t"<< f_measure << "\t"<< avg_class_accuracy << "\t\t"<< duration_sec;        		    
			

		}
		
		results_of_experiments << "\n---------------------------------------------------------------------------------------------------------";       
		results_of_experiments.close();
		results_of_experiments.clear();
    }
    else
    {
		ROS_INFO("File exist");
		string tmp;
		std::ifstream num_results_of_experiments;
		num_results_of_experiments.open (log_file_path.c_str());
		while (num_results_of_experiments.good()) 
		{
			std::getline (num_results_of_experiments, tmp);
			if(tmp.empty () || tmp.at (0) == '#') // Skip blank lines or comments
			continue;
			number_of_exist_experiments++;
		}
		num_results_of_experiments.close();
		exp_num = (number_of_exist_experiments)/2;
		ROS_INFO("number_of_exist_experiments = %i",exp_num );

		std::ofstream results_of_experiments;
		results_of_experiments.open (log_file_path.c_str(), std::ofstream::app);
		results_of_experiments.precision(4);

		if (object_descriptor == "GOOD")
		{
			results_of_experiments << "\n" << exp_num << "\t" << object_descriptor <<"\t\t"<< descriptor_param <<"\t" << K <<"\t"<< distance_function << "\t\t"<< f_measure << "\t"<< avg_class_accuracy << "\t\t"<< duration_sec;
		}	
		else if (object_descriptor == "VFH")
		{
			results_of_experiments << "\n" << exp_num << "\t" << object_descriptor <<"\t\t"<< descriptor_param <<"\t" << K <<"\t"<< distance_function << "\t\t"<< f_measure << "\t"<< avg_class_accuracy << "\t\t"<< duration_sec;
		}
		else if (object_descriptor == "ESF")
		{
			results_of_experiments << "\n" << exp_num << "\t" << object_descriptor <<"\t\t"<< "---" <<"\t" << K <<"\t"<< distance_function << "\t\t"<< f_measure << "\t"<< avg_class_accuracy << "\t\t"<< duration_sec;        		    
		}

		results_of_experiments << "\n---------------------------------------------------------------------------------------------------------";       
		results_of_experiments.close();
		results_of_experiments.clear();
    } 
   
	return 0;	
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int chiSquaredDistanceBetweenTwoObjectViewHistogram (SITOV objectViewHistogram1,
						    SITOV objectViewHistogram2, 
						    float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() ==  objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
		  if (objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i) > 0)
		  {
			  diffrence += pow( (objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i)) , 2) / 
				      (objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i)) ;
		  }
		}
		diffrence = 0.5 * diffrence;
		return(1);
	}
	else 
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int chiSquaredBasedObjectCategoryDistance( SITOV target,
		vector< SITOV > category_instances,
		float &minimumDistance, 
		int &best_matched_index, 
		PrettyPrint &pp)

{
	size_t category_size = category_instances.size();  
	best_matched_index=-1;//not matched

	if (category_size < 1)
	{
		pp.warn(std::ostringstream().flush() <<  "Error: Size of category is zero - could not compute objectCategoryDistance D(t,C)");
		// return 0;
	}
	else
	{
		// find the minimum distance between target object and category instances 
		std::vector<float> listOfDiffrence;
		float minimum_distance =10000000;
		for (size_t i=0; i<category_size; i++)
		{
			float tmp_diff =0;
			SITOV categoryInstance;
			categoryInstance = category_instances.at(i);

			chiSquaredDistanceBetweenTwoObjectViewHistogram(target,categoryInstance, tmp_diff);
			listOfDiffrence.push_back(tmp_diff);
			if (tmp_diff < minimum_distance)
			{
				minimum_distance=tmp_diff;
				best_matched_index=i;
			}    
		}
		minimumDistance=minimum_distance;
	}

	return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int diffrenceBetweenTwoObjectViewHistogram(SITOV objectViewHistogram1,
		SITOV objectViewHistogram2, 
		float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() ==  objectViewHistogram2.spin_image.size())
	{
		diffrence =0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
			diffrence += pow( (objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i)) , 2);
			// 	    diffrence += log (pow( (objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i)) , 2));
		}
		// 	diffrence = log (diffrence);
		return(1);
	}
	else 
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int euclideanBasedObjectCategoryDistance( SITOV target,
		vector< SITOV > category_instances,
		float &minimumDistance, 
		int &best_matched_index, 
		PrettyPrint &pp)

{
	size_t category_size = category_instances.size();  
	best_matched_index=-1;//not matched

	if (category_size < 1)
	{
		pp.warn(std::ostringstream().flush() <<  "Error: Size of category is zero - could not compute objectCategoryDistance D(t,C)");
		// return 0;
	}
	else
	{
		// find the minimum distance between target object and category instances 
		std::vector<float> listOfDiffrence;
		float minimum_distance =10000000;
		for (size_t i=0; i<category_size; i++)
		{
			float tmp_diff =0;
			SITOV categoryInstance;
			categoryInstance = category_instances.at(i);

			diffrenceBetweenTwoObjectViewHistogram(target,categoryInstance, tmp_diff);
			//pp.info(std::ostringstream().flush() <<"diffrenceBetweenTwoObjectViewHistogram [target, Instance "<< i<<"]= "<< tmp_diff);
			listOfDiffrence.push_back(tmp_diff);
			if (tmp_diff < minimum_distance)
			{
				minimum_distance=tmp_diff;
				best_matched_index=i;
			}    
		}
		//pp.info(std::ostringstream().flush() << "D(target,category) ="<< minimum_distance);

		minimumDistance=minimum_distance;
	}

	return 1;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
int kLdiffrenceBetweenTwoObjectViewHistogram(SITOV objectViewHistogram1,
					      SITOV objectViewHistogram2, 
					      double &diffrence)
{
	
	//https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	if (objectViewHistogram1.spin_image.size() ==  objectViewHistogram2.spin_image.size())
	{
		diffrence =10000000;
		double distance_P_Q =0;
		double distance_Q_P =0;
		
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
			if ((objectViewHistogram1.spin_image.at(i) == 0) || (objectViewHistogram2.spin_image.at(i) == 0) )
			{
				continue;
				//objectViewHistogram1.spin_image.at(i) = 0.00001; // maybe we should skip this element
			}
			distance_P_Q += objectViewHistogram1.spin_image.at(i) * log10 (objectViewHistogram2.spin_image.at(i)/objectViewHistogram1.spin_image.at(i));
			distance_P_Q += objectViewHistogram2.spin_image.at(i) * log10 (objectViewHistogram1.spin_image.at(i)/objectViewHistogram2.spin_image.at(i));
		}
		diffrence = -0.5 * (distance_P_Q + distance_Q_P);
		return(1);
	}
	else 
	{
		return(0);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int kLBasedObjectCategoryDistance( SITOV target,
		vector< SITOV > category_instances,
		float &minimumDistance, 
		int &best_matched_index, 
		PrettyPrint &pp)

{
	size_t category_size = category_instances.size();  
	best_matched_index=-1;//not matched

	if (category_size < 1)
	{
		pp.warn(std::ostringstream().flush() <<  "Error: Size of category is zero - could not compute objectCategoryDistance D(t,C)");
		// return 0;
	}
	else
	{
		// find the minimum distance between target object and category instances 
		std::vector<float> listOfDiffrence;
		float minimum_distance =10000000;
		for (size_t i=0; i<category_size; i++)
		{
			double tmp_diff = 0;
			SITOV categoryInstance;
			categoryInstance = category_instances.at(i);

			kLdiffrenceBetweenTwoObjectViewHistogram(target,categoryInstance, tmp_diff);
			
			listOfDiffrence.push_back(tmp_diff);
			if (tmp_diff < minimum_distance)
			{
				minimum_distance=tmp_diff;
				best_matched_index=i;
			}    
		}

		//ROS_INFO("\t\t[-]-objectCategoryDistance D(target,category) is: %f ", minimum_distance);
		//pp.info(std::ostringstream().flush() << "D(target,category) ="<< minimum_distance);

		minimumDistance=minimum_distance;
		//pp.printCallback();
	}

	return 1;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
void  confusionMatrixGenerator (  string true_category, string predicted_category, 
                                    std::vector<string> map_category_name_to_index,
                                    std::vector< std::vector <int> > &confusion_matrix )
{
    
    int true_index = -1;
    int predicted_index = -1;
    for (int i = 0; i<confusion_matrix.size(); i++ ) 
    {
        if (true_category.c_str() == map_category_name_to_index.at(i) )
            true_index = i;

        if (predicted_category.c_str() == map_category_name_to_index.at(i) )
            predicted_index = i;
    }

    if ((true_index != -1) && (predicted_index != -1))
    {
        confusion_matrix.at(true_index).at(predicted_index) ++;
    }
    else
    {   
        ROS_INFO("Error computing confusion matrix");
    }
    cout << "confusion_matrix [" << confusion_matrix.size() << "," << confusion_matrix.at(0).size() << "]= \n";
    
    for (int i = 0; i<confusion_matrix.size(); i++ ) 
    { 
		cout << map_category_name_to_index.at(i) << "\t\t";
        for (int j =0; j<confusion_matrix.at(i).size(); j++ ) 
             cout << confusion_matrix.at(i).at(j) << ",\t";
        cout << "\n";
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void findClosestCategory(vector<float> object_category_distances,
			  int &cat_index, float &mindist, PrettyPrint &pp, float &sigma_distance)
{

	// index = index;
	sigma_distance=0;
	size_t i;
	
	if (object_category_distances.size() < 1)
	{
		//ROS_ERROR("Error: Size of NormalObjectToCategoriesDistances is 0");
		pp.warn(std::ostringstream().flush() << "No categories known");
		cat_index = -1;
		return;
	}
	cat_index = 0;
	mindist = object_category_distances.at(0);
	for (i = 1; i < object_category_distances.size(); i++)
	{
		sigma_distance += object_category_distances.at(i);
		if (mindist > object_category_distances.at(i))
		{
			mindist = object_category_distances.at(i);
			cat_index = i;
		}
	}
	//pp.info(std::ostringstream().flush() << "Sum of distances =" <<sigma_distance << "; numcats=" << object_category_distances.size());
	//pp.info(std::ostringstream().flush() << "min distance =" << mindist);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int chiSquaredDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() ==  objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
		  if (objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i) > 0)
		  {
            // you just need to change this line of code
            diffrence += pow( (objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i)) , 2) / 
                    (objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i)) ;
		  }
		}
		diffrence = 0.5 * diffrence;
		return(1);
	}
	else 
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int symmetricKullbackLieblerDiverganceDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	//https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	if (objectViewHistogram1.spin_image.size() ==  objectViewHistogram2.spin_image.size())
	{
		diffrence =10000000;
		double distance_P_Q =0;
		double distance_Q_P =0;
		
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
			if ((objectViewHistogram1.spin_image.at(i) == 0) || (objectViewHistogram2.spin_image.at(i) == 0) )
			{
				continue;
				//objectViewHistogram1.spin_image.at(i) = 0.00001; // maybe we should skip this element
			}
			distance_P_Q += objectViewHistogram1.spin_image.at(i) * log10 (objectViewHistogram2.spin_image.at(i)/objectViewHistogram1.spin_image.at(i));
			distance_Q_P += objectViewHistogram2.spin_image.at(i) * log10 (objectViewHistogram1.spin_image.at(i)/objectViewHistogram2.spin_image.at(i));
		}
		diffrence = -0.5 * (distance_P_Q + distance_Q_P);
		return(1);
	}
	else 
	{
		return(0);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int kullbackLieblerDiverganceDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	//https://en.wikipedia.org/wiki/Kullback%E2%80%93Leibler_divergence
	if (objectViewHistogram1.spin_image.size() ==  objectViewHistogram2.spin_image.size())
	{
		diffrence =10000000;
		double distance_P_Q =0;
				
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
			if ((objectViewHistogram1.spin_image.at(i) == 0) || (objectViewHistogram2.spin_image.at(i) == 0) )
			{
				continue;
			}
			distance_P_Q += objectViewHistogram1.spin_image.at(i) * log10 (objectViewHistogram2.spin_image.at(i)/objectViewHistogram1.spin_image.at(i));
		}
		diffrence = -distance_P_Q ;
		return(1);
	}
	else 
	{
		return(0);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int motykaDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		float diffrence2 = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
            // you just need to change this line of code
            diffrence += std::max(objectViewHistogram1.spin_image.at(i), objectViewHistogram2.spin_image.at(i));
            diffrence2 += (objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i));
		}
		diffrence = diffrence / diffrence2;
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int divergenceDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		
			for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
			{
                // you just need to change this line of code					
                diffrence += (pow((objectViewHistogram1.spin_image.at(i)-objectViewHistogram2.spin_image.at(i)), 2))/(pow((objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i)), 2));
			}
			diffrence = 2*diffrence;
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int euclideanDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
            // you just need to change this line of code
            diffrence += pow((objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i)), 2);
		}
		diffrence = sqrt( diffrence);
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int intersectionDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
            // you just need to change this line of code
            diffrence += std::min(objectViewHistogram1.spin_image.at(i) , objectViewHistogram2.spin_image.at(i));
		}
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int manhattanDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
			// you just need to change this line of code
			diffrence += fabs(objectViewHistogram1.spin_image.at(i)-objectViewHistogram2.spin_image.at(i));
		}

		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////

int cosineDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
            // you just need to change this line of code
            diffrence += (objectViewHistogram1.spin_image.at(i)*objectViewHistogram2.spin_image.at(i));
		}
		diffrence = 1-(diffrence);
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int diceDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		float diffrence1 = 0;
			float diffrence2 = 0;
			for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
			{        
                // you just need to change this line of code
                diffrence += pow((objectViewHistogram1.spin_image.at(i)- objectViewHistogram2.spin_image.at(i)),2);
                diffrence1 += pow(objectViewHistogram1.spin_image.at(i),2);
                diffrence2 += pow(objectViewHistogram2.spin_image.at(i), 2);

			}
		diffrence = diffrence / (diffrence1 + diffrence2);
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int bhattacharyyaDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
        for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
        {
            // you just need to change this line of code
            diffrence += sqrt(objectViewHistogram1.spin_image.at(i) * objectViewHistogram2.spin_image.at(i));
        }
		diffrence = -log(diffrence);
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int sorensenDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		float diffrence1 = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
			// you just need to change this line of code
			diffrence += abs(objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i));
			diffrence1 += objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i);
		}
		diffrence = diffrence / diffrence1;
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int canberraDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{  
			if (objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i) > 0)
			{
				// you just need to change this line of code
				diffrence += (abs(objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i))) / (objectViewHistogram1.spin_image.at(i) + objectViewHistogram2.spin_image.at(i));
			}
		}
		
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int pearsonDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
			if (objectViewHistogram2.spin_image.at(i) != 0)
			{
				// you just need to change this line of code
				diffrence += (pow(objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i),2)) / (objectViewHistogram2.spin_image.at(i));
			}
		}
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int neymanDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{
			if (objectViewHistogram1.spin_image.at(i) != 0)
			{
				// you just need to change this line of code
				diffrence += (pow(objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i), 2)) / (objectViewHistogram1.spin_image.at(i));
			}
		}

		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int gowerDistanceBetweenTwoObjectViewHistograms(SITOV objectViewHistogram1, SITOV objectViewHistogram2, float &diffrence)
{
	if (objectViewHistogram1.spin_image.size() == objectViewHistogram2.spin_image.size())
	{
		diffrence = 0;
		for (size_t i = 0; i < objectViewHistogram1.spin_image.size(); i++)
		{        
            // you just need to change this line of code
            diffrence += abs(objectViewHistogram1.spin_image.at(i) - objectViewHistogram2.spin_image.at(i));
		}
        diffrence = diffrence / (float) objectViewHistogram1.spin_image.size();
		return(1);
	}
	else
	{
		ROS_INFO("\t\t[-]- object1 size = %ld", objectViewHistogram1.spin_image.size());
		ROS_INFO("\t\t[-]- object2 size = %ld", objectViewHistogram2.spin_image.size());
		ROS_ERROR("Can not compare two object view histograms with diffrent lenght");
		return(0);
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
enum distance_function_code {
    chiSquared,
    KLDivergance,
    symmetricKL,
    motyka,
    divergence,
    euclidean,
    intersection,
    manhattan,
    cosine,
    dice,
    bhattacharyya,
    sorensen,
    canberra,
    pearson,
    neyman,
    gower
};
////////////////////////////////////////////////////////////////////////////////////////////////////
distance_function_code hashTableForDistanceFunctions (std::string const& distance_function) {
    if (distance_function == "chiSquared") 
        return chiSquared;
    else if (distance_function == "KLDivergance") 
        return KLDivergance;
    else if (distance_function == "symmetricKL") 
        return symmetricKL;
    else if (distance_function == "motyka") 
        return motyka;
    else if (distance_function == "divergence") 
        return divergence;
    else if (distance_function == "euclidean") 
        return euclidean;
    else if (distance_function == "intersection") 
        return intersection;
    else if (distance_function == "manhattan") 
        return manhattan;
    else if (distance_function == "dice") 
        return dice;
    else if (distance_function == "bhattacharyya") 
        return bhattacharyya;
    else if (distance_function == "sorensen") 
        return sorensen;
    else if (distance_function == "canberra") 
        return canberra;
    else if (distance_function == "pearson") 
        return pearson;
    else if (distance_function == "neyman") 
        return neyman;
    else if (distance_function == "gower") 
        return gower;  
}

////////////////////////////////////////////////////////////////////////////////////////////////////
vector <float> objectCategoryDistance( SITOV target, 
										vector< SITOV > category_instances,
										float &minimumDistance, 
										int &best_matched_index,
										string distance_function,
										PrettyPrint &pp)
{
	size_t category_size = category_instances.size();  
	best_matched_index=-1;//not matched
	vector <float> distances;

	if (category_size < 1)
	{
		pp.warn(std::ostringstream().flush() <<  "Error: Size of category is zero - could not compute objectCategoryDistance D(t,C)");
		// return 0;
	}
	else
	{
		// find the minimum distance between target object and category instances 
		//std::cout << "test object = " << target.ground_truth_name << endl; 
		float minimum_distance = 10000000;
		for (size_t i = 0; i < category_size; i++)
		{
			float tmp_diff =0;
			SITOV categoryInstance;
			categoryInstance = category_instances.at(i);
			
			//std::cout << "train object = " << categoryInstance.ground_truth_name << endl; 

            switch (hashTableForDistanceFunctions(distance_function.c_str())) 
			{
                case chiSquared:
                    chiSquaredDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case KLDivergance:
                    kullbackLieblerDiverganceDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case symmetricKL:
                    symmetricKullbackLieblerDiverganceDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case motyka:
                    motykaDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case divergence:
                    divergenceDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case euclidean:
                    euclideanDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                case manhattan:
                    manhattanDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case intersection:
                    intersectionDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case cosine:
                    cosineDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case dice:
                    diceDistanceBetweenTwoObjectViewHistograms(target, categoryInstance, tmp_diff);
                    break;
                case bhattacharyya:
                    bhattacharyyaDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case sorensen:
                    sorensenDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case canberra:
                    canberraDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case pearson:
                    pearsonDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case neyman:
                    neymanDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
                case gower:
                    gowerDistanceBetweenTwoObjectViewHistograms (target, categoryInstance, tmp_diff);
                    break;
			}
			
			if (tmp_diff == 0)
			{
				// ROS_INFO( "size of target object = %d, size of training object = %d)", target.spin_image.size(), categoryInstance.spin_image.size());
				// ROS_INFO( "D(%s, %s) = %f", target.ground_truth_name.c_str(), categoryInstance.ground_truth_name.c_str(), tmp_diff);

				// ROS_INFO( "\n\nTarget object representation = ");

				// for (int i =0; i < target.spin_image.size(); i++ )
				// 	std::cout << target.spin_image.at(i) << ", ";

				// ROS_INFO( " \n\nTrain object representation = ");
				// for (int i =0; i < categoryInstance.spin_image.size(); i++ )
				// 	std::cout << categoryInstance.spin_image.at(i) << ", ";
				
				distances.push_back (999999999);
				continue;

			}


 			if (tmp_diff < minimum_distance)
			{
				minimum_distance=tmp_diff;
				best_matched_index=i;
				//ROS_INFO( "D(target, category) = %f", minimum_distance);
			}  
			distances.push_back (tmp_diff);
  
		}

		pp.info(std::ostringstream().flush() << "D(target,category) ="<< minimum_distance);
		minimumDistance=minimum_distance;
		//pp.printCallback();
	}

	return distances; // can be used for KNN classification
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int knnClassification(int K, vector<KNN_struct> data, float &minimum_distance, string &win_category)
{
	struct knn {
		float dist;
		string category_name;
	};

	vector<knn> KNN;
	for (size_t i = 0; i < K; i++)
	{
		knn tmp;
		tmp.dist = 99999999;
		tmp.category_name = "unknown";
		KNN.push_back(tmp);
	}
			
	//ROS_INFO ("############### KNN:");

	for (size_t i = 0; i < data.size(); i++)//category
	{
		for (size_t j = 0; j < K; j++)
		{
			for (int m = 0; m < K; m ++)
			{
				if (data.at(i).distances.at(j) <= KNN.at(m).dist )	
				{
					knn tmp;
					tmp.dist = data.at(i).distances.at(j);
					tmp.category_name = data.at(i).category_name;					
					KNN.insert(KNN.begin()+m, tmp);
					KNN.pop_back();
					//debug
					// cout << "[";
					// for (int n = 0; n < KNN.size(); n++)
					// {	
					// 	cout << "(" << KNN.at(n).dist << "," << KNN.at(n).category_name << "), ";
					// }
					// cout << "]\n";
					break;
				}
			}
		}
	}

	//ROS_INFO ("############### Voting:");
	string str ;
	for (size_t i = 0; i < K; i++)//category
	{
		str = str + KNN.at(i).category_name.c_str() + ", " ;
	}
	int major_vote = 0;
	minimum_distance = 9999999;
	win_category = "Unknown" ;

	for (size_t i = 0; i < KNN.size(); i++)
	{
		int count = 0;
		int nPos = str.find(KNN.at(i).category_name, 0); 
		while (nPos != string::npos)
		{
			count++;
			nPos = str.find(KNN.at(i).category_name, nPos + KNN.at(i).category_name.size());
		}
		//cout << "Number of times " <<KNN.at(i).category_name << " appears in ( "<< str << ")= " << count << endl; 
		if (count == K)
		{
			major_vote = count;
			win_category = KNN.at(i).category_name;
			minimum_distance = KNN.at(i).dist;
			break;
		}
		else if (count > 0) 
		{
			if (count > major_vote)	
			{
				major_vote = count;
				win_category = KNN.at(i).category_name;
				minimum_distance = KNN.at(i).dist;
			}
			else if ((count == major_vote) && (KNN.at(i).dist < minimum_distance))
			{
				major_vote = count;
				win_category = KNN.at(i).category_name;
				minimum_distance = KNN.at(i).dist;
			}
		}
	}
	cout << "\n\t [-] Major Voting: " << major_vote << " ---  win_category = "<< win_category <<
			" --- min_dist = " << minimum_distance << "\n" <<endl; 

	return (0);

}
////////////////////////////////////////////////////////////////////////////////////////////////////

void  plotConfusionMatrix(std::vector< std::vector <int> > confusion_matrix,
							std::vector<string> map_category_name_to_index, 
							string name_of_approach)
{
	//// adapted from https://github.com/vtshitoyan/plotConfMat
	string confusion_matrix_file = ros::package::getPath("rug_kfold_cross_validation")+ "/result/experiment_1/visualize_confusion_matrix.m";
	ofstream matlab_file;
	matlab_file.open (confusion_matrix_file.c_str(), std::ofstream::trunc);

	matlab_file << "clear; close all; clc; \n";
	matlab_file << "confmat = [ ";

	for (int i = 0; i < confusion_matrix.size(); i++ ) 
	{
		for (int j = 0; j < confusion_matrix.at(i).size()-1; j++ ) 
		{
			matlab_file << confusion_matrix.at(i).at(j) << ", ";  
		}
		matlab_file << confusion_matrix.at(i).at(confusion_matrix.at(i).size()-1) << ";\n";
	}
	
	matlab_file << "];\n\n% plotting\n";
	matlab_file << "plotConfusionMatrix(confmat, { ";
	for (int i =0; i < confusion_matrix.size(); i++ ) 
	{
		matlab_file << "'"<<map_category_name_to_index.at(i) << "', ";
	}
	matlab_file << "});";

	matlab_file <<"\n\n\n\n\n%%\nfunction plotConfusionMatrix(varargin)\n%";
	matlab_file <<"PLOTCONFMAT plots the confusion matrix with colorscale, absolute numbers\n%";
	matlab_file <<"and precision normalized percentages\n%\n%   usage: \n%   PLOTCONFMAT(confmat) plots the confmat with integers 1 to n as class labels\n%";
	matlab_file <<"PLOTCONFMAT(confmat, labels) plots the confmat with the specified labels\n%\n%   Arguments\n%   confmat:\t\t\ta square confusion matrix\n%";
	matlab_file <<"labels (optional):  vector of class labels\n\n";

	matlab_file <<"% number of arguments\n";
	matlab_file <<"switch (nargin)\n";
	matlab_file <<"\tcase 0\n";
	matlab_file <<"\t	confmat = 1;\n";
	matlab_file <<"\t	labels = {'1'};\n";
    matlab_file <<"\tcase 1\n";
	matlab_file <<"\t	confmat = varargin{1};\n";
	matlab_file <<"\t	labels = 1:size(confmat, 1);\n";
	matlab_file <<"\totherwise\n";
	matlab_file <<"\t	confmat = varargin{1};\n";
	matlab_file <<"\t	labels = varargin{2};\n";
	matlab_file <<"end\n\n";
	matlab_file <<"confmat(isnan(confmat))=0; % in case there are NaN elements\n";
	matlab_file <<"numlabels = size(confmat, 1); % number of labels\n\n";
	matlab_file <<"% calculate the percentage accuracies\n";
	matlab_file <<"confpercent = 100*confmat./sum(confmat, 2);\n\n";
	matlab_file <<"% plotting the colors\n";
	matlab_file <<"imagesc(confpercent);\n";
	matlab_file <<"title(sprintf('Accuracy: %.2f%% ("<< name_of_approach.c_str() <<")', 100*trace(confmat)/sum(confmat(:))));\n";
	matlab_file <<"ylabel('Predicted Category','FontSize',15); xlabel('Target Category','FontSize',15);\n\n";
	matlab_file <<"% set the colormap\n";
	matlab_file <<"colormap(flipud(hot));\n";
	matlab_file <<"colorbar;\n\n";

	matlab_file <<"% Create strings from the matrix values and remove spaces\n";
	matlab_file <<"textStrings = num2str([confpercent(:), confmat(:)], '%.1f%%\\n%d');\n";
	matlab_file <<"textStrings = strtrim(cellstr(textStrings));\n";
	matlab_file <<"\n% Create x and y coordinates for the strings and plot them\n";
	matlab_file <<"[x,y] = meshgrid(1:numlabels);\n";
	matlab_file <<"hStrings = text(x(:),y(:),textStrings(:), ...\n";
	matlab_file <<"'HorizontalAlignment','center');\n\n";
	matlab_file <<"% Get the middle value of the color range\n";
	matlab_file <<"midValue = mean(get(gca,'CLim'))\n\n";
	matlab_file <<"% Choose white or black for the text color of the strings so\n";
	matlab_file <<"% they can be easily seen over the background color\n";
	matlab_file <<"textColors = repmat(confpercent(:) > midValue,1,3);\n";
	matlab_file <<"set(hStrings,{'Color'},num2cell(textColors,2));\n\n";
	matlab_file <<"% Setting the axis labels\n";
	matlab_file <<"truesize([450 550]);\n";

	matlab_file <<"set(gca,'XTick',1:numlabels,...\n";
	matlab_file <<"\t	'XTickLabel',labels,...\n";
	matlab_file <<"\t	'YTick',1:numlabels,...\n";
	matlab_file <<"\t	'YTickLabel',labels,...\n";
	matlab_file <<"\t	'TickLength',[0 0]);\n";
	matlab_file <<"end\n";

	matlab_file.close();

}
////////////////////////////////////////////////////////////////////////////////////////////////////

struct RGBcolor{
  float r;
  float g;
  float b;
};

RGBcolor colorGenerating (int i)
{
	boost::shared_ptr<PointCloud<PointT> > point_cloud (new PointCloud<PointT>);
	PointT point;
	boost::shared_ptr<class_colormap_utils> cm;
	cm = (boost::shared_ptr<class_colormap_utils>) new class_colormap_utils(std::string("hot"),5, 1, false);
	vector <RGBcolor> color_code;
	RGBcolor black {0.1,0.1,0.1};
	RGBcolor Absolute_Zero {0, 0.28,0.73};

		
	//List of color are available at https://en.wikipedia.org/wiki/List_of_colors:_A%E2%80%93F  
	// RGBcolor Absolute_Zero {0, 0.28,0.73};
	color_code.push_back(Absolute_Zero);

	RGBcolor Acid_Green {0.69, 0.75, 0.10};
	color_code.push_back(Acid_Green);

	RGBcolor Aero_Blue {0.79,0.100,0.90};
	color_code.push_back(Aero_Blue);

	RGBcolor African_Violet{0.70,0.52,0.75};
	color_code.push_back(African_Violet);

	RGBcolor  Air_Superiority_Blue {0.45,0.63,0.76};
	color_code.push_back(Air_Superiority_Blue);

	RGBcolor  Apple_Green {0.55, 0.71, 0.0};
	color_code.push_back(Apple_Green);

	RGBcolor  Alabama_Crimson {0.69, 0, 0.16};
	color_code.push_back(Alabama_Crimson);

	RGBcolor  Alloy_Orange {0.45,0.63,0.76};
	color_code.push_back(Alloy_Orange);

	RGBcolor  Amaranth {0.90, 0.17, 0.31};
	color_code.push_back(Amaranth);

	RGBcolor  Amber {1, 0.75,0};
	color_code.push_back(Amber);

	RGBcolor  Amethyst {0.60, 0.40, 0.80};
	color_code.push_back(Amethyst);

	RGBcolor  Arctic_Lime {0.82, 1, 0.8};
	color_code.push_back(Arctic_Lime);

	RGBcolor  Arsenic {0.23, 0.27,	0.29};
	color_code.push_back(Arsenic);

	RGBcolor  Bisque {1, 0.89, 0.77};
	color_code.push_back(Bisque);

	RGBcolor  Bistre {0.24, 0.17, 0.12};
	color_code.push_back(Bistre);

	RGBcolor  Bistre_Brown {0.59, 0.64, 0.7};
	color_code.push_back(Bistre_Brown);

	RGBcolor  Bitter_Lemon {0.79, 0.88, 0.5};
	color_code.push_back(Bitter_Lemon);

	RGBcolor  Bitter_Lime {0.75, 1, 0};
	color_code.push_back(Bitter_Lime);

	RGBcolor  Bulgarian_Rose {0.28, 0.2, 0.3};
	color_code.push_back(Bulgarian_Rose);

	RGBcolor  Byzantine {0.74, 0.20, 0.64};
	color_code.push_back(Byzantine);

	RGBcolor Cadet_Blue {0.37, 0.62, 0.63};
	color_code.push_back(Cadet_Blue);

	RGBcolor Carrot_Orange {0.93, 0.57, 0.13};
	color_code.push_back(Carrot_Orange);

	RGBcolor  Castleton_Green {0, 0.34, 0.25};
	color_code.push_back(Castleton_Green);

	RGBcolor  Catawba {0.44, 0.21, 0.26};
	color_code.push_back(Catawba);

	RGBcolor  Cameo_Pink {0.94, 0.73, 0.80};
	color_code.push_back(Cameo_Pink);

	RGBcolor  Daffodil {1, 1, 0.19};
	color_code.push_back(Daffodil);

	RGBcolor  Cyan {0, 1, 1};
	color_code.push_back(Cyan);

	RGBcolor  Deep_Spring_Bud {33, 0.42, 0.18};
	color_code.push_back(Deep_Spring_Bud);

	RGBcolor  Diamond {73, 0.95 ,1};
	color_code.push_back(Diamond);

	RGBcolor  French_Beige {0.65, 0.48, 0.36};
	color_code.push_back(French_Beige);

	int color_code_size =30;
	if (i >= color_code_size)
	{
		div_t divresult;
		divresult = div (i, 30);		
		//divresult.quot, divresult.rem
		//ROS_INFO("Given color index is not exist, idx should be in [0-%d] range ", color_code.size()-1);
		return (color_code.at(divresult.rem));
	}

	// //test
	// for (float j = 0; j < color_code.size(); j++ )
	// {
	// 	point.x = j*0.1;
	// 	point.y = j*0.1;
	// 	point.z = j*0.1;
	// 	point.r = color_code.at(j).r*255;
	// 	point.g = color_code.at(j).g*255;
	// 	point.b = color_code.at(j).b*255;
	// 	point.a = 1;
	// 	point_cloud->push_back (point);	   
	// }

	// pcl::visualization::PCLVisualizer viewer ("Topics and point cloud");
	// viewer.addPointCloud (point_cloud, "topic point cloud");
	// viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 35, "topic point cloud");

	// viewer.setBackgroundColor (255, 255, 255);
	// while (!viewer.wasStopped ())
	// { viewer.spinOnce (100);}
  
	return (color_code.at(i));
}

///////////////////////////////////////////////////////////////////////////////////////////
int visualizingPointCloud(boost::shared_ptr<pcl::PointCloud<PointT> > point_cloud_in, 
					      string name_of_window, 
						  double duration_second = 0)
{

	// this is necessary to convert PointXYZRGBA to PointXYZRGB
	pcl::io::savePCDFile ("/tmp/test.pcd", *point_cloud_in, true);	
	boost::shared_ptr<PointCloud<pcl::PointXYZRGB> > point_cloud (new PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile <pcl::PointXYZRGB> ("/tmp/test.pcd", *point_cloud);
			
	ros::Time begin_process = ros::Time::now(); //start tic	
	// visualization point cloud
	pcl::visualization::PCLVisualizer viewer1 (name_of_window.c_str());
	viewer1.addPointCloud ( point_cloud, name_of_window.c_str());
	viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, name_of_window.c_str());
	viewer1.addCoordinateSystem (0.1);
	viewer1.setBackgroundColor (255, 255, 255);
	ros::Duration duration = ros::Time::now() - begin_process;
	double duration_sec = duration.toSec();
	
	if (duration_second == 0)
	{
		while (!viewer1.wasStopped ())
		{ viewer1.spinOnce (100);}	
	}
	else
	{
		while (duration_sec < duration_second)
		{ 
			duration = ros::Time::now() - begin_process;
			duration_sec = duration.toSec();
			viewer1.spinOnce (100);
		}
		viewer1.wasStopped ();
	}
	return 0;
}


#endif


