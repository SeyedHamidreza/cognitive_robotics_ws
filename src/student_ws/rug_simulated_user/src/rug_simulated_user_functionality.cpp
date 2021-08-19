// ############################################################################
//    
//   Created: 	1/09/2020
//   Author : 	Hamidreza Kasaei
//   Email  :	hamidreza.kasaei@rug.nl
//   Purpose: 	This program follows the teaching protocol and autonomously
//		interact with the system using teach, ask and correct actions. 
// 		For each newly taught category, the average sucess of the system
// 		should be computed. To do that, the simulated teacher repeatedly 
// 		picks object views of the currently known categories from a 
// 		database and presents them to the system for checking whether 
// 		the system can recognize them. If not, the simulated teacher provides
// 		corrective feedback.
//   		
//
//   	See https://www.ai.rug.nl/irl-lab/
// 
//   	(Copyright) University of Groningen - AI Dep.
// 
// ############################################################################

/* _______________________________
|                                 |
|           INCLUDES              |
|_________________________________| */

//ROS includes
#include <ros/ros.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ros/package.h>
#include <pcl/io/ply_io.h>
#include <tf/tf.h>


//system includes
#include <std_msgs/String.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <CGAL/Plane_3.h>
#include <algorithm>


//ros includes 
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

//perception db includes
#include <race_perception_msgs/perception_msgs.h>
#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>

//pcl includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

//package includes
#include <object_descriptor/object_descriptor_functionality.h>
#include <feature_extraction/spin_image.h>
#include <object_conceptualizer/object_conceptualization.h>
#include <race_perception_utils/print.h>

// define deep learning service here
#include <race_deep_learning_feature_extraction/deep_representation.h>
#include <rug_deep_feature_extraction/deep_representation.h>

// define topic modelling service here
#include <race_topic_modeling_services/topic_modelling.h>
#include <rug_kfold_cross_validation/rug_kfold_cross_validation_functionality.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

/* _____________________________
|                               |
|            constant           |
|_______________________________| */

// #define spin_image_width 4
// #define subsample_spinimages 0
// #define spin_image_support_lenght 0.05

#define recognitionThershold 20000

/* _______________________________
|                                 |
|         Global variable         |
|_________________________________| */
  
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBA T;  

PerceptionDB* _pdb; //initialize the class
int track_id_gloabal =1;
ofstream allFeatures;
std::string PCDFileAddressTmp;
int Hist_lenght = 0;

using namespace pcl;
using namespace std;
using namespace ros;

/////////////////////////////////////////////////////////////////////////////////////////////////////
int getAvergeRGBColorOfObject(boost::shared_ptr<PointCloud<PointT> > target_pc, 
											  float &red, 
											  float &green,
											  float &blue)
{
	//// get average object color in RGB 
	for (size_t i = 0; i < target_pc->points.size(); i++)
	{
		red += target_pc->points.at(i).r;
		green += target_pc->points.at(i).g;
		blue += target_pc->points.at(i).b;
	}
	
	////get average color
	red = red / (target_pc->points.size() + 1);
	green = green / (target_pc->points.size() + 1);
	blue = blue / (target_pc->points.size() + 1);

};

/////////////////////////////////////////////////////////////////////////////////////////////////////
int rgb2hsv(float r, float g, float b, float &h, float &s, float &v)
{
    
    double min, max, delta;

    min = r < g ? r : g;
    min = min  < b ? min  : b;

    max = r > g ? r : g;
    max = max  > b ? max  : b;

    v = max; 
    delta = max - min;
    if (delta < 0.00001)
    {
        s = 0;
        h = 0; // undefined, maybe nan?
        return 1;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        s = (delta / max);                  // s
    } 
	else 
	{
		// if max is 0, then r = g = b = 0              
		// s = 0, h is undefined
		s = 0.0;
		h = 0; // its now undefined
		return 1;
    }
    if( r >= max )// > is bogus, just keeps compilor happy
        h = ( g - b ) / delta; // between yellow & magenta
    else
    if( g >= max )
        h = 2.0 + ( b - r ) / delta;  // between cyan & yellow
    else
        h = 4.0 + ( r - g ) / delta;  // between magenta & cyan

    h *= 60.0; // degrees

    if( h < 0.0 )
        h += 360.0;

    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
string fixedLength (string name , size_t length)
{
	while (name.length() < length)
    { 
		name += " ";
    }
    return (name);
} 

/////////////////////////////////////////////////////////////////////////////////////////////////////
string extractObjectNameSimulatedUser (string object_name_orginal )
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

/////////////////////////////////////////////////////////////////////////////////////////////////////
string extractCategoryName (string InstancePath )
{
	
	// ROS_INFO("\t\t ****************** instance_path = %s", InstancePath.c_str());
    string categoryName="";	    
    int ffind = InstancePath.find("//")+2;  
 	// ROS_INFO("\t\t left = %d", ffind);
    int lfind =  InstancePath.find("_Cat");       
 	// ROS_INFO("\t\t right = %d", lfind);
    for (int i=0; i<(lfind-ffind); i++)
    {
		categoryName += InstancePath.at(i+ffind);
    }
	// ROS_INFO("\t\t categoryName = %s", categoryName);

    return (categoryName);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int putObjectViewSpinImagesInSpecificCategory(std::string cat_name, unsigned int cat_id, 
					    unsigned int track_id, unsigned int view_id, 
					    boost::shared_ptr< vector <SITOV> > SpinImageMsg,
					    double &Cat_ICD )
{
	PrettyPrint pp;
	SITOV msg_in;
	RTOV _rtov;
	_rtov.track_id = track_id;
	_rtov.view_id = view_id;
	
	for (size_t i = 0; i < SpinImageMsg->size(); i++)
	{
	    msg_in = SpinImageMsg->at(i);
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
	ROS_INFO("\t\t[-] v_key: %s, track_id: %i, view_id: %i", v_key.c_str(), track_id, view_id);

	//Put one view to the db
	_pdb->put(v_key, v_s);
	
	////////////////////////////////////////////////////////////////
	//Put OC with one view
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
	
	double New_ICD = 0;
	intraCategoryDistance(category_instances, New_ICD, pp);
	// ROS_INFO("\t\t[-]- ICD = %f", New_ICD);
	_oc.icd = New_ICD;
	
	
	Cat_ICD = New_ICD;
	oc_size = ros::serialization::serializationLength(_oc);

	boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
	PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::serialize(oc_buffer, _oc, oc_size);	
	leveldb::Slice ocs((char*)oc_buffer.get(), oc_size);
	_pdb->put(oc_key, ocs);
	return (0);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int addObjectViewHistogramInSpecificCategory(std::string cat_name,
					      unsigned int cat_id, 
					      unsigned int track_id,
					      unsigned int view_id, 
					      SITOV objectViewHistogram,
					      PrettyPrint &pp )
{
    //PrettyPrint pp;
    SITOV msg_in;
    RTOV _rtov;
    _rtov.track_id = track_id;
    _rtov.view_id = view_id;

	msg_in = objectViewHistogram;
	msg_in.spin_img_id = 1;

	uint32_t sp_size = ros::serialization::serializationLength(msg_in);

	boost::shared_array<uint8_t> sp_buffer(new uint8_t[sp_size]);
	PerceptionDBSerializer<boost::shared_array<uint8_t>, SITOV>::serialize(sp_buffer, msg_in, sp_size);
	leveldb::Slice sp_s((char*)sp_buffer.get(), sp_size);
	std::string sp_key = _pdb->makeSIKey(key::SI, track_id, view_id, 1 );

	//Put slice to the db
	_pdb->put(sp_key, sp_s); 

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
	_oc.icd = 0.00001;

    oc_size = ros::serialization::serializationLength(_oc);

    pp.info(std::ostringstream().flush() << _oc.cat_name.c_str() << " category has " << _oc.rtov_keys.size() << " objects.");
    //pp.info(std::ostringstream().flush() << "ICD for " << _oc.cat_name.c_str() << " category updated. New ICD is: "<< _oc.icd);

    boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::serialize(oc_buffer, _oc, oc_size);	
    leveldb::Slice ocs((char*)oc_buffer.get(), oc_size);
    _pdb->put(oc_key, ocs);

    return (1);
}

int addObjectViewHistogramInSpecificCategoryDeepLearning(std::string cat_name, unsigned int cat_id, 
															unsigned int track_id, unsigned int view_id, 
															SITOV objectViewHistogram , PrettyPrint &pp
															)
{
	//ROS_INFO("****========*********========*********========*********========*********========*****");

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
    float New_ICD = 1;
    _oc.icd = New_ICD;
    oc_size = ros::serialization::serializationLength(_oc);

    pp.info(std::ostringstream().flush() << _oc.cat_name.c_str() << " category has " << _oc.rtov_keys.size() << " objects.");
    //pp.info(std::ostringstream().flush() << "ICD for " << _oc.cat_name.c_str() << " category updated. New ICD is: "<< _oc.icd);

    boost::shared_array<uint8_t> oc_buffer(new uint8_t[oc_size]);
    PerceptionDBSerializer<boost::shared_array<uint8_t>, ObjectCategory>::serialize(oc_buffer, _oc, oc_size);	
    leveldb::Slice ocs((char*)oc_buffer.get(), oc_size);
    _pdb->put(oc_key, ocs);
	//ROS_INFO("****========***** update a CATEGORY to PDB took = %f", (ros::Time::now() - start_time).toSec());
	//ROS_INFO("****========*********========*********========*********========*********========*****");
    return (1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int introduceNewInstanceUsingAHandCraftedDescriptor( string dataset_path,
													 std::string PCDFileAddress,
													 unsigned int track_id,												
													 string object_descriptor,                                                      
													 int number_of_bins, 
													 double normal_estimation_radius,
													 PrettyPrint &pp )
{

	unsigned int view_id = 1; 
	int adaptive_support_lenght = 0; // 0 true 1 false
	double global_image_width = 0.2;
	int threshold = 10; // threshold on number of positive and negative points

    //ROS_INFO ("name of given object view = %s",PCDFileAddress.c_str());
    string categoryName = extractCategoryName(PCDFileAddress);

    if(PCDFileAddress.empty () || PCDFileAddress.at (0) == '#') // Skip blank lines or comments
    {
		return 0;
    }
    
    PCDFileAddress = dataset_path +"/"+ PCDFileAddress.c_str();

    //load a PCD object  
    boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);    
	try 
	{ 
		io::loadPCDFile <PointXYZRGBA> (PCDFileAddress.c_str(), *target_pc);
	} 
	catch (const std::exception& e) 
	{ 
		ROS_ERROR("\t\t[-] could not read given object %s :", PCDFileAddress.c_str());					
		return(0);
	}
	ROS_INFO("\t\t[1]-adding a new instance : %s", PCDFileAddress.c_str());
      
	SITOV object_representation;

 	if (object_descriptor == "GOOD")
    {
        /* _________________________________
        |                                   |
        |  option1: GOOD shape descriptor   |
        |___________________________________| */
    
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
	
		for (size_t i = 0; i < object_description.size(); i++)
		{
			object_representation.spin_image.push_back(object_description.at(i));
		}
		object_representation.ground_truth_name = PCDFileAddress.c_str();

    }
    else if (object_descriptor == "ESF")
    {
        /* ________________________________
        |                                  |
        |  option2: ESF shape descriptor   |
        |__________________________________| */

		pcl::PointCloud<pcl::ESFSignature640>::Ptr esf (new pcl::PointCloud<pcl::ESFSignature640> ());
		estimateESFDescription (target_pc, esf);
		size_t esf_size = sizeof(esf->points.at(0).histogram)/sizeof(float);
		for (size_t i = 0; i < esf_size ; i++)
		{
			object_representation.spin_image.push_back( esf->points.at(0).histogram[i]);
		}
		ROS_INFO("ESF_size = %ld",object_representation.spin_image.size());
		object_representation.ground_truth_name = PCDFileAddress.c_str();

    }
    else if (object_descriptor == "VFH")
    {
        /* _________________________________
        |                                   |
        |  option3: VFH shape descriptor    |
        |___________________________________| */
		
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfh (new pcl::PointCloud<pcl::VFHSignature308> ());
		estimateViewpointFeatureHistogram(  target_pc, 
											normal_estimation_radius,
											vfh);

		size_t vfh_size = sizeof(vfh->points.at(0).histogram)/sizeof(float);
		
		for (size_t i = 0; i < vfh_size ; i++)
		{
			object_representation.spin_image.push_back( vfh->points.at(0).histogram[i]);
		}
		//ROS_INFO("VFH_size = %ld",object_representation.spin_image.size());
		object_representation.ground_truth_name = PCDFileAddress.c_str();
        
    }
    else if (object_descriptor == "GRSD")
    {
        /* _________________________________
        |                                   |
        |  option4: GRSD shape descriptor   |
        |___________________________________| */

		pcl::PointCloud<pcl::GRSDSignature21>::Ptr grsd (new pcl::PointCloud<pcl::GRSDSignature21> ());
		estimateGRSDDescription(target_pc, normal_estimation_radius, grsd);
		
		size_t grsd_size = sizeof(grsd->points.at(0).histogram)/sizeof(float);
		for (size_t i = 0; i < grsd_size ; i++)
		{
			object_representation.spin_image.push_back( grsd->points.at(0).histogram[i]);
		}
		//ROS_INFO("GRSD_size = %ld",object_representation.spin_image.size());		
		object_representation.ground_truth_name = PCDFileAddress.c_str();

    }
    else 
    {
        ROS_ERROR("The object descriptor has not been implemented yet!!!");
        return 1;    
    }
    
  
    ROS_INFO("\t\t[-] size of object view histogram %ld",object_representation.spin_image.size());
    
    addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, view_id, object_representation , pp);	
    return (1);
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int selectAnInstancefromSpecificCategory( unsigned int category_index, 
										  unsigned int &instance_number, 
										  string &Instance)
{
    std::string dataset_path;
	ros::param::get("/perception/dataset_path", dataset_path);
	std::string path =  dataset_path +"/Category/Category.txt";
	// ROS_INFO("path = %s",path.c_str());
    
    //ROS_INFO("TEST");
	// ROS_INFO("\n");
    ROS_INFO("\t\t[-] category index = %d",category_index);
    ROS_INFO("\t\t[-] instance_number = %d",instance_number);

    std::ifstream listOfObjectCategoriesAddress (path.c_str());
    std::string categoryAddresstmp;
    std::string categoryName;

    unsigned int cat_index = 0;
    while ((listOfObjectCategoriesAddress.good()) && (cat_index < category_index))
    {
		std::getline (listOfObjectCategoriesAddress, categoryAddresstmp);
		if(categoryAddresstmp.empty () || categoryAddresstmp.at (0) == '#') // Skip blank lines or comments
			continue;

		if (cat_index < category_index)
			cat_index++;
    }
    
    if (cat_index != category_index)
    {
		ROS_INFO("\t\t[-] the file doesn't exist - condition : cat_index != category_index, cat_index = %i", cat_index);
		return -1;
    }
    
	path = dataset_path +"/"+ categoryAddresstmp.c_str();
    
    std::ifstream categoyInstances (path.c_str());
    std::string PCDFileAddressTmp;
    
    unsigned int inst_number = 0;
    while ((categoyInstances.good ()) && (inst_number < instance_number))// read instances of a category 
    {	
		std::getline (categoyInstances, PCDFileAddressTmp);
		if(PCDFileAddressTmp.empty () || PCDFileAddressTmp.at (0) == '#') // Skip blank lines or comments
			continue;
		if (inst_number < instance_number)
	    	inst_number ++;
    }
    if (inst_number < instance_number)
    {
		ROS_INFO("\t\t[-] category path = %s", path);
		ROS_INFO("\t\t[-] the file doesn't exist - condition : inst_number < instance_number -- inst_number = %i", inst_number);
		return -1;
    }
    
    Instance=PCDFileAddressTmp;
    instance_number++;
    return 0;
    
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int introduceNewCategoryUsingAHandCraftedDescriptor(string dataset_path,
													int class_index,
													unsigned int &track_id,
													unsigned int &instance_number,
													string object_descriptor,                                                      
													int number_of_bins, 
													double normal_estimation_radius,
													string fname, 
													int k)
{
	ROS_INFO(" *****************************************************************");
    ROS_INFO(" **** teaching a new object category using %d instances ****", k);
    ROS_INFO(" *****************************************************************");

	if (k < 3) k = 3;
	string instance_path;
	for(int i = 0; i < k ; i++)  // 2 instances would be enough
	{
	    if (selectAnInstancefromSpecificCategory(class_index, instance_number, instance_path)==-1)
	    {	
			ROS_ERROR("\t\t[-]-The object view or category does not exist");
			return -1;// ERROR : file doesn't exist
	    }
	    ROS_INFO("\t\t[-]-Instance path = %s", instance_path.c_str());
	    int cat_id = 1; // cat_id is always 1 beacuse we use cat_name key:<cat_name><cat_id>
	    int view_id = 1; // view_id is always 1 beacuse we use TID key:<TID><VID>
	      
	    PrettyPrint pp;
		introduceNewInstanceUsingAHandCraftedDescriptor( dataset_path,
														 instance_path,
														 track_id,												
														 object_descriptor,                                                      
														 number_of_bins, 
														 normal_estimation_radius,
														 pp );
														
	    
	    track_id++;
		ROS_INFO("\n");

	}
	// extracting the category name 
	string categoryName=extractCategoryName(instance_path);
	ROS_INFO("\n extractCategoryName %s", categoryName.c_str()); 
 	//report_category_introduced(fname,categoryName.c_str());
	return 0;
    
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int introduceNewInstanceGOOD ( string dataset_path,
								std::string PCDFileAddress,
								unsigned int cat_id, 
								unsigned int track_id,
								unsigned int view_id, 
								int adaptive_support_lenght,
								double global_image_width,
								int threshold,
								int number_of_bins,
								PrettyPrint &pp
								)
{

    ROS_INFO ("\t\t[-] name of given object view = %s",PCDFileAddress.c_str());
    string categoryName = extractCategoryName(PCDFileAddress);

    
    if(PCDFileAddress.empty () || PCDFileAddress.at (0) == '#') // Skip blank lines or comments
    {
		return 0;
    }
    
    PCDFileAddress = dataset_path +"/"+ PCDFileAddress.c_str();

    //load a PCD object  
    boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
    if (io::loadPCDFile <PointXYZRGBA> (PCDFileAddress.c_str(), *target_pc) == -1)
    {	
	    ROS_ERROR("\t\t[-]-Could not read given object %s :",PCDFileAddress.c_str());
	    return(0);
    }
    else
    {
	    ROS_INFO("\t\t[1]-adding a new instance : %s", PCDFileAddress.c_str());
    }
       

    /* ________________________________________________
    |                                                 |
    |  Compute GOOD Description for given point cloud |
    |_________________________________________________| */
  
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
    ROS_INFO("\nsize of object view histogram %ld",object_representation.spin_image.size());
    
    addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
    return (1);
    
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int introduceNewInstanceGOODPlusColor ( string dataset_path,
										std::string PCDFileAddress,
										unsigned int cat_id, 
										unsigned int track_id,
										unsigned int view_id, 
										int adaptive_support_lenght,
										double global_image_width,
										int threshold,
										int number_of_bins,
										double weight_color,
										string color_space,
										PrettyPrint &pp	)
{

    ROS_INFO ("name of given object view = %s",PCDFileAddress.c_str());
    string categoryName = extractCategoryName(PCDFileAddress);

    
    if(PCDFileAddress.empty () || PCDFileAddress.at (0) == '#') // Skip blank lines or comments
    {
		return 0;
    }
    
    PCDFileAddress = dataset_path +"/"+ PCDFileAddress.c_str();

    //load a PCD object  
    boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);
    if (io::loadPCDFile <PointXYZRGBA> (PCDFileAddress.c_str(), *target_pc) == -1)
    {	
	    ROS_ERROR("\t\t[-]-Could not read given object %s :",PCDFileAddress.c_str());
	    return(0);
    }
    else
    {
	    ROS_INFO("\t\t[1]-adding a new instance : %s", PCDFileAddress.c_str());
    }
       

    /* ________________________________________________
    |                                                 |
    |  Compute GOOD Description for given point cloud |
    |_________________________________________________| */
  
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
    ROS_INFO("\nsize of object view histogram %ld",object_representation.spin_image.size());
    
	ROS_INFO("color_space.c_str() = %s", color_space.c_str());

	float red = 0, green = 0, blue = 0;
	getAvergeRGBColorOfObject(target_pc, red, green, blue);

	////color information // TODO should be function
	if (strcmp(color_space.c_str(), "YUV") == 0)
	{
		// https://stackoverflow.com/questions/5392061/algorithm-to-check-similarity-of-colors
		float y = 0, u = 0, v = 0;
		y = red * 0.299 + green * 0.587 + blue * 0.114;
		u = (red * -0.168736 + green * -0.331264 + blue * 0.50) + 128;
		v = (red * 0.5 + green * -0.418688 + blue * -0.081312) + 128;
		
		object_representation.spin_image.push_back(y/255);
		object_representation.spin_image.push_back(u/255);
		object_representation.spin_image.push_back(v/255);

	}
	else if (strcmp(color_space.c_str(), "HSV") == 0)
	{
		// https://stackoverflow.com/questions/35113979/calculate-distance-between-colors-in-hsv-space
		// The "Hue" elements are in the range [0,360]. With the above formula, you can compute a distance between two hues. This distance is in the range [0,180]. Dividing the distance by 180.0 will result in a value in [0,1]
		// The "Saturation" elements are in the range [0,1]. The (absolute) difference between two saturations will also be in the range [0,1].
		// The "Value" elements are in the range [0,255]. The absolute difference between two values will thus be in the range [0,255] as well. Dividing this difference by 255.0 will result in a valu

		float h = 0, s = 0, v = 0;
		rgb2hsv(red, green, blue, h, s, v);
		object_representation.spin_image.push_back(h);
		object_representation.spin_image.push_back(s);
		object_representation.spin_image.push_back(v);
	}
	else // RGB
	{
		//// normalization	
		object_representation.spin_image.push_back(red/255);
		object_representation.spin_image.push_back(green/255);
		object_representation.spin_image.push_back(blue/255);
	}
    addObjectViewHistogramInSpecificCategory(categoryName, 1, track_id, 1, object_representation , pp);	
    return (1);
    
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool isImageEmpty( cv_bridge::CvImagePtr cv_ptr)
{
	float *p;
	
	for (int i=1; i < cv_ptr->image.rows; i++)
	{
		p = cv_ptr->image.ptr< float >( i ); // get a row of the image
		if ((p[i] != 0) || (p[cv_ptr->image.rows - i] != 0) ) ///check pixel value
		{
			return false;
		}
	}
	return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
int introduceNewInstanceRGBDDeepLearningUsingGOOD ( string dataset_path,
													std::string PCDFileAddress,
													unsigned int cat_id, 
													unsigned int track_id,
													unsigned int view_id, 
													int adaptive_support_lenght,
													double global_image_width,
													int threshold,
													int number_of_bins,
													ros::ServiceClient deep_learning_server,
													PrettyPrint &pp )
{

    ROS_INFO ("\t\t[-] name of given object view = %s",PCDFileAddress.c_str());
    string categoryName = extractCategoryName(PCDFileAddress);
    
    if(PCDFileAddress.empty () || PCDFileAddress.at (0) == '#' || categoryName == "Category//Unk") // Skip blank lines or comments
    {
		return -1;
    }
    
    PCDFileAddress = dataset_path +"/"+ PCDFileAddress.c_str();

    //load a PCD object  
    boost::shared_ptr<PointCloud<PointT> > target_pc (new PointCloud<PointT>);      
	try 
	{ 
		io::loadPCDFile <PointXYZRGBA> (PCDFileAddress.c_str(), *target_pc) ;
	} 
	catch (const std::exception& e) 
	{ 
		ROS_ERROR("\t\t[-] could not read given object %s :", PCDFileAddress.c_str());					
	    return(-1);
	}	
	
    /* ________________________________________________
    |                                                 |
    |  Compute GOOD Description for given point cloud |
    |_________________________________________________| */

	boost::shared_ptr<pcl::PointCloud<PointT> > pca_object_view (new PointCloud<PointT>);
	boost::shared_ptr<PointCloud<PointT> > pca_pc (new PointCloud<PointT>); 
	vector < boost::shared_ptr<pcl::PointCloud<PointT> > > vector_of_projected_views;
	double largest_side = 0;
	int  sign = 1;
	vector <float> view_point_entropy;
	string list_of_sorted_projected_plane;
	Eigen::Vector3f center_of_bbox;
	vector< float > object_description;          

	computeDepthBasedGoodDescriptorAndSaveDepthAndRGBProjections( target_pc,
																adaptive_support_lenght,
																global_image_width,
																threshold,
																number_of_bins, // orthographic_image_resolution,
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
	cv_ptr->image = cv::imread(img_name.c_str(), CV_LOAD_IMAGE_COLOR);      
	// ROS_INFO ("img_name = %s", img_name.c_str());

	cv_bridge::CvImagePtr cv_ptr_depth(new cv_bridge::CvImage);
	cv_ptr_depth->encoding = "bgr8";
	img_name = "/tmp/"+ view_with_max_enntropy +"_merged.jpg";
	cv_ptr_depth->image = cv::imread(img_name.c_str(), CV_LOAD_IMAGE_COLOR);      

	// if(isImageEmpty(cv_ptr_depth))
	// 	return -1;

	/// fill deep msg
	SITOV deep_object_representation;
	rug_deep_feature_extraction::deep_representation srv;
	srv.request.good_representation = object_representation.spin_image;
	srv.request.RGB_image = *cv_ptr->toImageMsg();
	srv.request.depth_image = *cv_ptr_depth->toImageMsg();

	/// call deep learning service  
	if (deep_learning_server.call(srv))
	{
		//pp.info(std::ostringstream().flush() << "################ receive server responce with size of " << srv.response.deep_representation.size() );
		//ROS_INFO("################ receive server responce with size of %ld", srv.response.deep_representation.size() );
		if (srv.response.deep_representation.size() < 1)
			ROS_ERROR("Failed to call deep learning service");
			
		for (size_t i = 0; i < srv.response.deep_representation.size(); i++)
		{
			deep_object_representation.spin_image.push_back(srv.response.deep_representation.at(i));
		}
	}
	else
	{
		ROS_ERROR("Failed to call deep learning service");
	}
	
	addObjectViewHistogramInSpecificCategoryDeepLearning(categoryName, 1, track_id, 1, deep_object_representation , pp);	
    return (1);
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int NumberofCategoriesinDataset (string dataset_path)
{   
    //Dataset Path
    string path = dataset_path+ "Category/Category_orginal.txt";
    ROS_INFO("\t\t[-]- database path = %s", path.c_str());
    std::ifstream listOfObjectCategories (path.c_str(), std::ifstream::in);

    int number_of_categories =0;
    size_t total_number_of_instances = 0;

    while(listOfObjectCategories.good ())
    {
	string categoryAddress;
	std::getline (listOfObjectCategories, categoryAddress);
	if(categoryAddress.empty () || categoryAddress.at (0) == '#') // Skip blank lines or comments
	{
	    continue;
	}
	number_of_categories ++;
    }
    
    return (number_of_categories);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

vector <int> generateSequence (int n)
{
    /* initialize random seed: */
    srand (time(NULL));
    vector <int> sequence;
    while( sequence.size() < n )
    {    
	/* generate random number between 1 and 10: */
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
	// test :
	//     vector <int> test;
	//     test = generateSequence(15);
	//     for (int i = 0; i< test.size(); i++)
	//     {
	// 		printf(" %i,",test.at(i));
	//     }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesInstances (string path)
{    
 	
	std::string dataset_path;
	ros::param::get("/perception/dataset_path", dataset_path);
	
	std::string path1;
	path1 =  dataset_path + path;
	// ROS_INFO("path1 = %s",path.c_str());
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
    
	// ROS_INFO("\n number_of_instances= %i \n",number_of_exist_instances);
    
    InstanceAddresstmp = "";
    vector <int> instances_sequence = generateSequence (number_of_exist_instances);
    std::ofstream instances;

	string path2 =  dataset_path + path;
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int generateRrandomSequencesCategories (int RunCount)
{

    ROS_INFO("\n\n");
    ROS_INFO(" *****************************************************************");
    ROS_INFO(" ****  shuffling data may take a few seconds, please wait!!!  ****");
    ROS_INFO(" *****************************************************************");

 	std::string dataset_path;
	ros::param::get("/perception/dataset_path", dataset_path);
	std::string path =  dataset_path + "/Category/Category_orginal.txt";
	// ROS_INFO("path = %s",path.c_str());

    std::ifstream listOfObjectCategoriesAddress (path.c_str());
    // ROS_INFO ("path = %s", path.c_str());

    string categoryAddresstmp = "";
    unsigned int number_of_exist_categories = 0;
    while (listOfObjectCategoriesAddress.good()) 
    {
		std::getline (listOfObjectCategoriesAddress, categoryAddresstmp);
		if(categoryAddresstmp.empty () || categoryAddresstmp.at (0) == '#') // Skip blank lines or comments
			continue;
		number_of_exist_categories++;
    }
        	
        	
    ROS_INFO ("number of category = %i", number_of_exist_categories);
    vector <int> categories_sequence = generateSequence (number_of_exist_categories);
    
    std::ofstream categoies;
    string path2 = dataset_path +"/Category/Category.txt";

    categoies.open (path2.c_str(), std::ofstream::out);
    for (int i =0; i < categories_sequence.size(); i++)
    {
		std::ifstream listOfObjectCategories (path.c_str());
		int j = 0;
		while ((listOfObjectCategories.good()) && (j < categories_sequence.at(i)))
		{
			std::getline (listOfObjectCategories, categoryAddresstmp);
			generateRrandomSequencesInstances(categoryAddresstmp.c_str());
			j++;
		}
		categoryAddresstmp.resize(categoryAddresstmp.size()-12);
		categoryAddresstmp += ".txt";
		categoies << categoryAddresstmp.c_str()<<"\n";
		}
    return 0 ;
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float compute_precision_of_last_3n (vector <int> recognition_results ,
				     int number_of_taught_categories)
{
    ROS_INFO("\t\t[-]- ^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&");
    ROS_INFO("\t\t[-]- Inside compute_precision_of_last_3n function");
    ROS_INFO("\t\t[-]- size of recognition result = %d",recognition_results.size() );
    if (recognition_results.size() <1)
    {
	printf(" Error: size of recognition result array is %i",recognition_results.size() );
	return 0;
    }
    for (int i =0; i< recognition_results.size(); i++)
    {
	    printf("%d, ",recognition_results.at(i) );
    }
    printf("\n");
    
    float precision = 0;
    int TP=0; int FP=0; int FN=0;
    
    int win_size = 3*number_of_taught_categories;
    ROS_INFO("\ni start from %d till %d",recognition_results.size() - win_size -1, recognition_results.size()-1 );
    ROS_INFO("windows size = %d",win_size );
    ROS_INFO("number_of_taught_categories = %d",number_of_taught_categories );

    
    for (int i = recognition_results.size()-1; i > recognition_results.size() - win_size ; i--)
    {
	int result = recognition_results.at(i);
// 	printf("%d, ",result );
	if (result==1)
	{
	    TP++;
	}
	else if (result==2)
	{
	    FP++;
	}
	else if (result==3)
	{
	    FN++;
	}
	else if (result==4)
	{
	    FP++;FN++;
	}
    }
    
    ROS_INFO("\t\t[-]- number of TP= %d, FP= %d, FN=%d", TP,FP,FN); 
    float Precision = TP/double (TP+FP);
    ROS_INFO("\t\t[-]- Precision = %f", Precision); 
    
    
//     cout << "\nTP =" <<TP;
//     cout << "\nFP =" <<FP;
//     cout << "\nFN =" <<FN;		
//     char ch;
//     cin >>ch;
    
    return(Precision);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float computeF1OfLast3n (vector <int> recognition_results ,
				     int number_of_taught_categories)
{
    ROS_INFO("\t\t[-]- ^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&");
    ROS_INFO("\t\t[-]- Inside compute_precision_of_last_3n function");
    ROS_INFO("\t\t[-]- size of recognition result = %d",recognition_results.size() );
    if (recognition_results.size() <1)
    {
	printf(" Error: size of recognition result array is %i",recognition_results.size() );
	return 0;
    }
    for (int i =0; i< recognition_results.size(); i++)
    {
	    printf("%d, ",recognition_results.at(i) );
    }
    printf("\n");
    
    float precision = 0;
    int TP = 0; int FP = 0; int FN = 0;
    
    int win_size = 3*number_of_taught_categories;
    ROS_INFO("\ni start from %d till %d",recognition_results.size() - win_size -1, recognition_results.size()-1 );
    ROS_INFO("windows size = %d",win_size );
    ROS_INFO("number_of_taught_categories = %d",number_of_taught_categories );

    
    for (int i = recognition_results.size()-1; i > recognition_results.size() - win_size ; i--)
    {
	int result = recognition_results.at(i);
	// printf("%d, ",result );
	if (result==1)
	{
	    TP++;
	}
	else if (result==2)
	{
	    FP++;
	}
	else if (result==3)
	{
	    FN++;
	}
	else if (result==4)
	{
	    FP++;FN++;
	}
    }
    
    
    
    ROS_INFO("\t\t[-]- number of TP= %d, FP= %d, FN=%d", TP,FP,FN); 
    
    
    float Precision = TP/double (TP+FP);
    float Recall = TP/double (TP+FN);    
    float F1 = 2 * (Precision * Recall )/(Precision + Recall );
    ROS_INFO("\t\t[-]- F1 = %f", F1); 

    // cout << "\nTP =" <<TP;
    // cout << "\nFP =" <<FP;
    // cout << "\nFN =" <<FN;		
    // char ch;
    // cin >>ch;
    
    return(F1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int compute_Precision_Recall_Fmeasure_of_last_3n (vector <int> recognition_results ,
				     int number_of_taught_categories, 
				     float &Precision, float &Recall, float &F1)
{
    // ROS_INFO("\t\t[-]- ^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&^&");
    // ROS_INFO("\t\t[-]- Inside compute_precision_of_last_3n function");
    // ROS_INFO("\t\t[-]- size of recognition result = %d",recognition_results.size() );
    if (recognition_results.size() <1)
    {
	printf(" Error: size of recognition result array is %i",recognition_results.size() );
	return 0;
    }
    for (int i =0; i< recognition_results.size(); i++)
    {
	    // printf("%d, ",recognition_results.at(i) );
    }
    // printf("\n");
    
    float precision = 0;
    int TP=0; int FP=0; int FN=0;
    
    int win_size = 3*number_of_taught_categories;
    // ROS_INFO("\ni start from %d till %d",recognition_results.size() - win_size -1, recognition_results.size()-1 );
    // ROS_INFO("windows size = %d",win_size );
    // ROS_INFO("number_of_taught_categories = %d",number_of_taught_categories );

    
    for (int i = recognition_results.size()-1; i > recognition_results.size() - win_size ; i--)
    {
		int result = recognition_results.at(i);
		// printf("%d, ",result );
		if (result==1)
		{
			TP++;
		}
		else if (result==2)
		{
			FP++;
		}
		else if (result==3)
		{
			FN++;
		}
		else if (result==4)
		{
			FP++;FN++;
		}
    }
    
    
    
    // ROS_INFO("\t\t[-]- number of TP= %d, FP= %d, FN=%d", TP,FP,FN); 
    
    
    Precision = TP/double (TP+FP);
    Recall = TP/double (TP+FN);    
    F1 = 2 * (Precision * Recall )/(Precision + Recall );
    // ROS_INFO("\t\t[-]- F1 = %f", F1); 
    
    
    // cout << "\nTP =" <<TP;
    // cout << "\nFP =" <<FP;
    // cout << "\nFN =" <<FN;		
    // char ch;
    // cin >>ch;
    
    return(F1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void monitorPrecision (string precision_file, float Precision )
{
    std::ofstream PrecisionMonitor;
    PrecisionMonitor.open (precision_file.c_str(), std::ofstream::app);
    PrecisionMonitor.precision(4);
    PrecisionMonitor << Precision<<"\n";
    PrecisionMonitor.close();
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void monitorF1VsLearnedCategory (string f1_learned_caegory, int TP, int FP, int FN )
{
    double Precision, Recall, F1;
    if ((TP+FP)!=0)
    {
	Precision = TP/double (TP+FP);
    }
    else
    {
	Precision = 0;
    }		
    if ((TP+FN)!=0)
    {
	Recall = TP/double (TP+FN);
    }
    else
    {
	Recall = 0;
    }
    if ((Precision + Recall)!=0)
    {
	  F1 = 2 * (Precision * Recall )/(Precision + Recall );
    }
    else
    {
	F1 = 0;
    }  
  
    std::ofstream f1_vs_learned_caegory;
    f1_vs_learned_caegory.open (f1_learned_caegory.c_str(), std::ofstream::app);
    f1_vs_learned_caegory.precision(4);
    f1_vs_learned_caegory << F1<<"\n";
    f1_vs_learned_caegory.close();
    
}

void reportCurrentResults(int TP, int FP, int FN, string fname, bool global)
{
	double Precision = TP/double (TP+FP);
	double Recall = TP/double (TP+FN);
	
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file.precision(4);
	if (global)
		Result_file << "\n\n\t******************* Global *********************";
	else	Result_file << "\n\n\t******************* Lastest run ****************";
	Result_file << "\n\t\t - True  Positive = "<< TP;
	Result_file << "\n\t\t - False Positive = "<< FP;
	Result_file << "\n\t\t - False Negative = "<< FN;
	Result_file << "\n\t\t - Precision  = "<< Precision;
	Result_file << "\n\t\t - Recall = "<< Recall;
	Result_file << "\n\t\t - F1 = "<< 2 * (Precision * Recall )/(Precision + Recall );
	Result_file << "\n\n\t************************************************\n\n";
	
	// Result << "\n\n\t***********average_class_precision**************\n\n";
	// Result << "\n\t\t - average_class_precision = "<< average_class_precision_value;
	
	Result_file << "\n------------------------------------------------------------------------------------------------------------------------------------";
	Result_file.close();
}

void report_category_introduced(string fname, string cat_name)
{
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file << "\n\t********************************************";
	Result_file << "\n\t\t - "<< cat_name.c_str() <<" Category Introduced";
	Result_file << "\n\t********************************************";
	Result_file.close();
}

void report_precision_of_last_3n(string fname, double Precision)
{
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file << "\n\t*********** Precision of last 3n **************";
	Result_file << "\n\t\t - precision = "<< Precision;
	Result_file << "\n\t********************************************";
	Result_file.close();
}


void reportF1OfLast3n(string fname, double F1)
{
	std::ofstream Result_file;
	Result_file.open (fname.c_str(), std::ofstream::app);
	Result_file << "\n\t*********** F1 of last 3n **************";
	Result_file << "\n\t\t - F1 = "<< F1;
	Result_file << "\n\t********************************************";
	Result_file.close();
}

int reportExperimentResult (vector <float> average_class_precision,
			     int number_of_stored_instances, 
			     int number_of_taught_categories,  
			     string fname, ros::Duration duration)
{
   
    float average_class_precision_value =0;
    for (int i =0; i<average_class_precision.size(); i++)
    {
	average_class_precision_value+=average_class_precision.at(i); 	    
    }
    average_class_precision_value=average_class_precision_value/average_class_precision.size();
    double duration_sec = duration.toSec();
    
    std::ofstream Result;
    Result.open (fname.c_str(), std::ofstream::app);
    Result.precision(4);
    // Result << "\n\t -Note: the experiment is terminated because there is not\n\t\tenough test data to continue the evaluation\n";
    Result << "\n\n\t************** Expriment Result ****************";
    Result << "\n\t\t - Average_class_precision = "<< average_class_precision_value;
    Result << "\n\t\t - All stored instances = "<< number_of_stored_instances;
    Result << "\n\t\t - Number of taught categories = "<< number_of_taught_categories;
    Result << "\n\t\t - Average number of instances per category = "<< float (number_of_stored_instances) / float (number_of_taught_categories);	
    Result << "\n\t\t - This expriment took " << duration_sec << " secs";
    Result << "\n\n\t************************************************\n\n";
    Result.close();
    return (0);
}

///////////////////////////////////////////////////////////////////

int introduceNewCategoryRGBDDeepLearningUsingGOOD( string dataset_path,
												int class_index,
												unsigned int &track_id,
												unsigned int &instance_number,
												string fname, 
												int adaptive_support_lenght,
												double global_image_width,
												int threshold,
												int number_of_bins, 
												ros::ServiceClient deep_learning_server,
												int k)
{

	ROS_INFO(" *****************************************************************");
    ROS_INFO(" **** teaching a new object category using %d instances ****", k);
    ROS_INFO(" *****************************************************************");

	if (k < 3) k = 3;
	string instance_path;
	for(int i = 0; i < k ; i++)  // 2 instances would be enough
	{
	    // selectAnInstancefromSpecificCategory(class_index, instance_number, instance_path);
	    if (selectAnInstancefromSpecificCategory(class_index, instance_number, instance_path)==-1)
	    {	
			ROS_ERROR("\t\t[-] the object view or category does not exist");
			i -=1;
			continue;
			//return -1;// ERROR : file doesn't exist
	    }
	    //ROS_INFO("\t\t[-] Instance path %s", instance_path.c_str());
	    int cat_id = 1; // cat_id is always 1 beacuse we use cat_name key:<cat_name><cat_id>
	    int view_id = 1; // view_id is always 1 beacuse we use TID key:<TID><VID>
	      
	    PrettyPrint pp;
	    if (introduceNewInstanceRGBDDeepLearningUsingGOOD ( dataset_path,
													instance_path,
													cat_id, track_id, view_id,
													adaptive_support_lenght,
													global_image_width,
													threshold,
													number_of_bins,
													deep_learning_server,
													pp ) == -1)
		{	
			i -=1;
			continue;
	    }									
	    
	    track_id++;
	    //view_id ++; // in this implementation we consider VID as a constant
		cout << endl;
	}
	// extracting the category name 
	string categoryName=extractCategoryName(instance_path);
	//ROS_INFO("\n extractCategoryName %s", categoryName.c_str()); 
 	//report_category_introduced(fname,categoryName.c_str());
	return 0;
   
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int introduceNewCategoryGOOD(string dataset_path,
			  int class_index,
			  unsigned int &track_id,
			  unsigned int &instance_number,
			  string fname, 
			  int adaptive_support_lenght,
			  double global_image_width,
			  int threshold,
			  int number_of_bins
 			)
{
	string instance_path;
	for(int i = 0; i < 3 ; i++)  // 2 instances would be enough
	{
	    if (selectAnInstancefromSpecificCategory(class_index, instance_number, instance_path)==-1)
	    {	
			ROS_ERROR("\t\t[-]-The object view or category does not exist");
			return -1;// ERROR : file doesn't exist
	    }
	    ROS_INFO("\t\t[-]-Instance path = %s", instance_path.c_str());
	    int cat_id = 1; // cat_id is always 1 beacuse we use cat_name key:<cat_name><cat_id>
	    int view_id = 1; // view_id is always 1 beacuse we use TID key:<TID><VID>
	      
	    PrettyPrint pp;
	    introduceNewInstanceGOOD ( dataset_path,
					instance_path,
					cat_id, track_id, view_id,
					adaptive_support_lenght,
					global_image_width,
					threshold,
					number_of_bins,
					pp );
	    
	    track_id++;
	}
	// extracting the category name 
	string categoryName=extractCategoryName(instance_path);
	ROS_INFO("\n extractCategoryName %s", categoryName.c_str()); 
 	//report_category_introduced(fname,categoryName.c_str());
	return 0;

    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int introduceNewCategoryGOODPlusColor(string dataset_path,
										int class_index,
										unsigned int &track_id,
										unsigned int &instance_number,
										string fname, 
										int adaptive_support_lenght,
										double global_image_width,
										int threshold,
										int number_of_bins,
										double weight_color,
										string color_space = "RGB")
{
	string instance_path;
	for(int i = 0; i < 3 ; i++)  // 2 instances would be enough
	{
	    if (selectAnInstancefromSpecificCategory(class_index, instance_number, instance_path)==-1)
	    {	
			ROS_ERROR("\t\t[-]-The object view or category does not exist");
			return -1;// ERROR : file doesn't exist
	    }
	    ROS_INFO("\t\t[-]-Instance path = %s", instance_path.c_str());
	    int cat_id = 1; // cat_id is always 1 beacuse we use cat_name key:<cat_name><cat_id>
	    int view_id = 1; // view_id is always 1 beacuse we use TID key:<TID><VID>
	      
	    PrettyPrint pp;
	    introduceNewInstanceGOODPlusColor ( dataset_path,
											instance_path,
											cat_id, track_id, view_id,
											adaptive_support_lenght,
											global_image_width,
											threshold,
											number_of_bins,
											weight_color,
											color_space,
											pp );

	    track_id++;
	}
	// extracting the category name 
	string categoryName=extractCategoryName(instance_path);
	ROS_INFO("\n extractCategoryName %s", categoryName.c_str()); 
 	//report_category_introduced(fname,categoryName.c_str());
	return 0;    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void plotSimulatedTeacherProgressInMatlab( int RunCount, float P_Threshold, string precision_file)
{
 
    // If the text inside the plot is not well-appear, please install the font in ubuntu using the following commands.
    // sudo apt-get install xfonts-75dpi
    // sudo apt-get install xfonts-75dpi
    
    char run_count [10];
	vector<float> acc;

    sprintf( run_count, "%d",RunCount );
    string path = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/graph1.m";
        
    // ROS_INFO("\t\tpath = %s", path.c_str());

    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "close all;\nfigure;\nhold on;\ngrid on;";
    matlabFile << "\nset(gca,'LineStyleOrder', '-');";
    matlabFile << "\nPrecision= [";
    
    std::string value;
    std::ifstream ReadPrecisionMonitor (precision_file.c_str());
    std::getline (ReadPrecisionMonitor, value);
    matlabFile << value;
	acc.push_back(atof (value.c_str()));

    int iteration = 1;
    while (ReadPrecisionMonitor.good())
    {
		std::getline (ReadPrecisionMonitor, value);
		matlabFile << ", "<< value;
		acc.push_back(atof (value.c_str()));
		iteration++;
    }
    
    int remain = iteration % 10;
    matlabFile << "];\naxis([0,"<< iteration + 10 - remain <<",0,1.2]);\nplot (Precision, 'LineWidth',2);";
    matlabFile << "\nxlabel('Question / Correction Iterations','FontSize',15);\nylabel('Protocol Accuracy','FontSize',15);";

    matlabFile << "\nset(gca,'LineStyleOrder', '--');";
    //draw a threshold line
    matlabFile << "\nline([0 "<< iteration + 10 - remain <<"],["<< P_Threshold << " " <<P_Threshold <<"] ,'Color',[0 0 0 0.3], 'LineWidth',1);";
    matlabFile << "\ntext(" << iteration - remain - 10 << "," << P_Threshold + 0.05 <<",'Thereshold', 'FontSize',12, 'Interpreter','Latex');";

    iteration = 0;
    string path_tmp = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/Category_Introduced.txt";
    std::ifstream read_category_introduced (path_tmp.c_str());

	std::string dataset_path;
	ros::param::get("/perception/dataset_path", dataset_path);

    string path_tmp2 = dataset_path + "/Category/Category.txt";
    std::ifstream category_name (path_tmp2.c_str());
    string cat_name ="";
    
    value = "";
    string value_tmp;
    // draw a line 
    std::getline (read_category_introduced, value_tmp);
    matlabFile << "\nline(["<< iteration<< ","<< iteration <<"] ,[0.05,1],'Color',[1 0 0], 'LineWidth',1);";
    
    // add category name to the graph
    std::getline (category_name, cat_name);
    cat_name = extractCategoryName(cat_name);
    
    int lfind =  cat_name.find("_");
    if (lfind > 0) 
		cat_name.replace(lfind, 1,1, '-');


    float text_Y_pos = 0.05;
    bool flg=true;
    matlabFile << "\ntext("<<iteration<<".5, 0.05 ,'"<< cat_name <<"' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex' ,'BackgroundColor', [1,0,0,0.2]);";
        
    std::getline (read_category_introduced, value_tmp);
    matlabFile << "\nline(["<< iteration<< ","<< iteration <<"] ,[0.1,1],'Color',[1 0 0], 'LineWidth',1);";	    
    std::getline (category_name, cat_name);
    
    cat_name = extractCategoryName(cat_name);
    lfind =  cat_name.find("_");
    if (lfind > 0) 
		cat_name.replace(lfind, 1,1, '-');
    
    matlabFile << "\ntext("<<iteration<<".5, 0.1,'"<< cat_name <<"' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex', 'BackgroundColor', [1,0,0,0.2] );";

    iteration = 1;
    float y = 0.15;
    while (read_category_introduced.good())
    {

		std::getline (read_category_introduced, value);
		if (strcmp(value.c_str(),"1") == 0)
		{
			std::getline (category_name, cat_name);

			cat_name = extractCategoryName(cat_name);
			lfind = cat_name.find("_");
			if (lfind > 0) 
			{
				cat_name.replace(lfind, 1,1, '-');
			}

			if (acc.at(iteration-1) == 0)
			{
				matlabFile << "\nline([" << iteration << "," << iteration <<"] ,["<< 1 - y <<", 0],'Color',[1 0 0], 'LineWidth',1);";	    
				matlabFile << "\ntext(" << iteration <<".5," << 1 - y <<" ,'"<< cat_name <<"' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex', 'BackgroundColor', [1,0,0,0.2]);";
			}
			else
			{
				matlabFile << "\nline(["<< iteration<< ","<< iteration <<"] ,["<< y <<", 1],'Color',[1 0 0], 'LineWidth',1);";	    
				matlabFile << "\ntext("<<iteration<<".5,"<< y <<" ,'"<< cat_name <<"' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex', 'BackgroundColor', [1,0,0,0.2]);";
			}
			if (iteration == 1) 
				iteration ++ ;
			


			if (y <=0.55)
			{
				y+=0.05;
			}
			else
			{
				y=0.05;
			}
		}
		else
		{
			iteration++;
		}
		
    }
    matlabFile.close();
    // ROS_INFO("\t\tMATLAB file created...");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void plotLocalF1VsNumberOfLearnedCategoriesInMatlab( int RunCount, 
													 float P_Threshold, 
													 string local_F1_vs_learned_category)
{
    // If the text inside the plot is not well-appear, please install the font in ubuntu using the following commands.
    // sudo apt-get install xfonts-75dpi
    
    char run_count [10];
    sprintf( run_count, "%d",RunCount );
    string path = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/local_F1_vs_learned_category.m";
        
    // ROS_INFO("\t\tpath = %s", path.c_str());

    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "close all;\nfigure ();\nhold on;\ngrid on;";
    matlabFile << "\nset(gca,'LineStyleOrder', '--');";
    matlabFile << "\ntext(1,0.685 ,'Threshold' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex');";

    matlabFile << "\nlocalF1= [";
    
    std::string value;
    std::ifstream Readlocal_F1 (local_F1_vs_learned_category.c_str());
    std::getline (Readlocal_F1, value);
    matlabFile << value;

    int iteration = 1;
    while (Readlocal_F1.good())
    {
		std::getline (Readlocal_F1, value);
		if(value.empty ()) // Skip blank lines or comments
			continue;
			
		matlabFile <<", " << value ;
		iteration++;
    }
    
    int remain = iteration % 5;
    matlabFile << "];\nline([0 "<< iteration + 5 - remain <<"],["<< P_Threshold << " " <<P_Threshold <<"] ,'Color',[0 0 0], 'LineWidth',1);";
    matlabFile << "\naxis([0,"<< iteration + 5 - remain <<",0.5,1.05]);"; 
    matlabFile << "\nplot(1:size(localF1,2), localF1(1:size(localF1,2)),'-.O', 'Color',[1 0 1], 'LineWidth',2.);";
    matlabFile << "\nxlabel('Number of Learned Categories','FontSize',15);";
    matlabFile << "\nylabel('Protocol Accuracy','FontSize',15);";

    matlabFile.close();
    ROS_INFO("\t\tMATLAB file created...");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void plotGlobalF1VsNumberOfLearnedCategoriesInMatlab(int RunCount, 
								    				  string global_F1_vs_learned_category)
{
    // If the text inside the plot is not well-appear, please install the font in ubuntu using the following commands.
    // sudo apt-get install xfonts-75dpi
    // sudo apt-get install xfonts-75dpi
    
    char run_count [10];
    sprintf( run_count, "%d",RunCount );
    string path = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/global_F1_vs_learned_category.m";
        
    // ROS_INFO("\t\tpath = %s", path.c_str());

    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "close all;\nfigure ();\nhold on;\ngrid on;";
    matlabFile << "\nset(gca,'LineStyleOrder', '--');";
    // matlabFile << "\ntext(1,0.685 ,'Threshold' , 'FontSize',15,'FontWeight','bold','Interpreter','Latex');";

    matlabFile << "\nglobalF1= [";
    
    std::string value;
    std::ifstream ReadGlobalF1 (global_F1_vs_learned_category.c_str());
    std::getline (ReadGlobalF1, value);
    matlabFile << value;

    int iteration = 1;
    while (ReadGlobalF1.good())
    {
		std::getline (ReadGlobalF1, value);
		if(value.empty ()) // Skip blank lines or comments
			continue;
		//  ROS_INFO("value = %s", value.c_str());
		matlabFile <<", " << value ;
		iteration++;
    }
    
    int remain = iteration % 5;
    matlabFile << "];\naxis([0,"<< iteration + 5 - remain <<",0.5,1.05]);"; 
    matlabFile << "\nplot(1:size(globalF1,2), globalF1(1:size(globalF1,2)),'-.O', 'Color',[0 0 1], 'LineWidth',2.);";
    matlabFile << "\nxlabel('Number of Learned Categories','FontSize',15);";
    matlabFile << "\nylabel('Global Classification Accuracy','FontSize',15);";
    matlabFile.close();
    // ROS_INFO("\t\tMATLAB file created...");
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void plotNumberOfLearnedCategoriesVsIterationsInMatlab( int RunCount, 
														string Number_of_learned_categories_vs_Iterations)
{
 
    // If the text inside the plot is not well-appear, please install the font in ubuntu using the following commands.
    // sudo apt-get install xfonts-75dpi
    // sudo apt-get install xfonts-75dpi
    
    char run_count [10];
    sprintf( run_count, "%d",RunCount );
    string path = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/number_of_learned_categories_vs_Iterations.m";
        
    // ROS_INFO("\t\tpath = %s", path.c_str());

    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "figure ();\nhold on;\ngrid on;";
    matlabFile << "\nset(gca,'LineStyleOrder', '-');";
    matlabFile << "\nNLI= [";
    
    int iteration = 1;
    std::string value;
    std::ifstream ReadNLI (Number_of_learned_categories_vs_Iterations.c_str());
    std::getline (ReadNLI, value);
    matlabFile << iteration;

    // int iteration = 2;
    while (ReadNLI.good())
    {
	std::getline (ReadNLI, value);
	if(value.empty ()) // Skip blank lines or comments
	    continue;	
	if (strcmp(value.c_str(),"1")==0)
	{
	    matlabFile << "," << iteration ;	    
	    iteration++;
	}
	else
	{
	    iteration++;
	}
    }
    matlabFile << "];";
    matlabFile << "\nplot(NLI(1:size(NLI,2)), 1:size(NLI,2), '--O', 'Color',[1 0 0], 'LineWidth',2.)";
    matlabFile << "\nxlabel('Question / Correction Iterations','FontSize',15);";
    matlabFile << "\nylabel('Number of Learned Categories','FontSize',15);";
    matlabFile.close();
    // ROS_INFO("\t\tMATLAB file created...");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void plotNumberOfStoredInstancesPerCategoryInMatlab( vector <ObjectCategory> list_of_object_category)
{
 
	string path = ros::package::getPath("rug_simulated_user")+ "/result/experiment_1/number_of_stored_instaces_per_category.m";
    ofstream matlabFile;
    matlabFile.open (path.c_str(), std::ofstream::out);
    matlabFile.precision(4);
    matlabFile << "figure ();\nhold on;\ngrid on;";
    matlabFile << "\nNIC= [";
	
	std::string dataset_path;
	ros::param::get("/perception/dataset_path", dataset_path);

	string path_tmp = dataset_path+ "/Category/Category.txt";
    std::ifstream category_name (path_tmp.c_str());
    string cat_name ="";

	int counter = 0;
	while ((category_name.good()) && 
			(counter < list_of_object_category.size()))
    {
		int idx = 0;
		std::getline (category_name, cat_name);
 	   	cat_name = extractCategoryName(cat_name);
		while ((list_of_object_category.at(idx).cat_name.c_str() != cat_name) && 
		 		(idx < list_of_object_category.size()))
		{
			idx ++;
		}

		if (counter < list_of_object_category.size()-1)
		{
			matlabFile << list_of_object_category.at(idx).rtov_keys.size()<<",";
		}
		else
		{
			matlabFile << list_of_object_category.at(idx).rtov_keys.size()<<"];\n";
		}		
		counter ++;
	}
	
	matlabFile << "list = {'";
	std::ifstream category_name_tmp (path_tmp.c_str());
	for (size_t i = 0; i < list_of_object_category.size(); i++) // retrieves all categories from perceptual memory
	{
		int idx = 0;
		std::getline (category_name_tmp, cat_name);
 	   	cat_name = extractCategoryName(cat_name);
		int lfind =  cat_name.find("_");
    	if (lfind > 0) 
			cat_name.replace(lfind, 1,1, '-');

		if (i < list_of_object_category.size()-1)
		{
			matlabFile << cat_name<<"', '";
		}
		else
		{
			matlabFile << cat_name<<"'};\n";
		}			
	}
	matlabFile << "categories = categorical(list,list);\n";
	matlabFile << "bar (categories, NIC);\n";
    matlabFile << "ylabel('Number of Stored Instances','FontSize',15);";
    matlabFile.close();
    ROS_INFO("\t\t[-] MATLAB files have been created ...");

    ROS_INFO("\t\t[-] Cleaning log files ...");

	string system_command;

	// system_command = "rm " + ros::package::getPath("rug_simulated_user") + "/result/experiment_1/Category.txt";
	// system( system_command.c_str());

	system_command= "rm " + ros::package::getPath("rug_simulated_user") + "/result/experiment_1/Category_Introduced.txt";
	system( system_command.c_str());
	
	system_command= "rm " + ros::package::getPath("rug_simulated_user") + "/result/experiment_1/f1_vs_learned_category.txt";
	system( system_command.c_str());

	system_command= "rm " + ros::package::getPath("rug_simulated_user") + "/result/experiment_1/local_f1_vs_learned_category.txt";
	system( system_command.c_str());

	system_command= "rm " + ros::package::getPath("rug_simulated_user") + "/result/experiment_1/PrecisionMonitor.txt";
	system( system_command.c_str());

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int IntroduceNewInstanceVFH ( std::string PCDFileAddress,
			    unsigned int cat_id, 
			    unsigned int track_id,
			    unsigned int view_id
			    )
{

    // string categoryName = PCDFileAddress;
    // categoryName.resize(13); 

    string categoryName = extractCategoryName(PCDFileAddress);
    if(PCDFileAddress.empty () || PCDFileAddress.at (0) == '#' || categoryName == "Category//Unk") // Skip blank lines or comments
    {
		return 0;
    }
    
	std::string dataset_path;
	ros::param::get("/perception/dataset_path", dataset_path);

    PCDFileAddress = dataset_path +"/"+ PCDFileAddress.c_str();

    //load a PCD object  
    boost::shared_ptr<PointCloud<PointT> > PCDFile (new PointCloud<PointT>);
    if (io::loadPCDFile <PointXYZRGBA> (PCDFileAddress.c_str(), *PCDFile) == -1)
    {	
	    ROS_ERROR("\t\t[-]-Could not read given object %s :",PCDFileAddress.c_str());
	    return(0);
    }
    else
    {
	    ROS_INFO("\t\t[1]-Loaded a point cloud: %s", PCDFileAddress.c_str());
    }
    
    //compute Spin Image for given point clould
    //Declare a boost share ptr to the SITOV msg
    boost::shared_ptr< vector <SITOV> > objectViewVFHs;
    objectViewVFHs = (boost::shared_ptr< vector <SITOV> >) new (vector <SITOV>);
    
    //Call the library function for estimateVFH
    estimateVFH( PCDFile, 
					0.01 /*downsampling_voxel_size*/, 
					0.05 /*normal_estimation_radius*/,
					objectViewVFHs,
					0);

    double Cat_ICD=0.0001;
    putObjectViewSpinImagesInSpecificCategory(categoryName,cat_id,track_id,view_id,objectViewVFHs,Cat_ICD);

    ROS_INFO("\t\t[-]-%s created...",categoryName.c_str());
        
    return (0);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int introduceNewCategoryVFH(int class_index,
			    unsigned int &track_id,
			    unsigned int &instance_number,
			    string fname
			   )
{
	string instance_path;
	for(int i = 0; i < 3 ; i++)  // 2 instances would be enough
	{
	    // selectAnInstancefromSpecificCategory(class_index, instance_number, instance_path);
	    if (selectAnInstancefromSpecificCategory(class_index, instance_number, instance_path)==-1)
	    {	
		ROS_ERROR("\t\t[-]-The object view or category does not exist");
		return -1;// ERROR : file doesn't exist
	    }
	    ROS_INFO("\t\t[-]-Instance number %ld", instance_number);
	    int cat_id = 1; // cat_id is always 1 beacuse we use cat_name key:<cat_name><cat_id>
	    int view_id = 1; // view_id is always 1 beacuse we use TID key:<TID><VID>
	    IntroduceNewInstanceVFH(instance_path, cat_id, track_id, view_id); 
	    track_id++;
	    //view_id ++; // in this implementation we consider VID as a constant
	}
	// extracting the category name 
	string categoryName=extractCategoryName(instance_path);
	ROS_INFO("\n extractCategoryName %s", categoryName.c_str()); 
 	report_category_introduced(fname,categoryName.c_str());
	return 0;
}


bool fexists(std::string filename) 
{
  ifstream ifile(filename.c_str());
  return ifile.good();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int sum_all_experiments_results ( int iterations,
									float Success_Precision		,			    
									float average_class_precision_value,
									float number_of_stored_instances, 
									int number_of_taught_categories,
									string name_of_approach )
{
    vector <float> tmp;
    tmp.push_back(float (iterations)); 
    tmp.push_back(float (number_of_taught_categories)); 
    tmp.push_back(float (number_of_stored_instances)/(float) number_of_taught_categories); 
    tmp.push_back(Success_Precision); 
    tmp.push_back(average_class_precision_value); 

    std::string sumResultsOfExperiments;
    sumResultsOfExperiments = ros::package::getPath("rug_simulated_user") + "/result/sum_all_results_of_"+name_of_approach+"_experiments.txt";
    ROS_INFO("results of expriments file path = %s",sumResultsOfExperiments.c_str() );
    int exp_num =1;
    if (!fexists(sumResultsOfExperiments.c_str()))
    {
		// ROS_INFO("File not exist");	
		std::ofstream sum_results_of_experiments;
		sum_results_of_experiments.open (sumResultsOfExperiments.c_str(), std::ofstream::out);
		sum_results_of_experiments.precision(4);
		sum_results_of_experiments <<iterations <<"\n"<< number_of_taught_categories <<"\n"<< number_of_stored_instances/(float) number_of_taught_categories <<"\n"<< Success_Precision<< "\n"<< average_class_precision_value;
		sum_results_of_experiments.close();
    }
	else
    {
		// ROS_INFO("File exist");
		string tmp_value;
		std::ifstream results_of_experiments;
		results_of_experiments.open (sumResultsOfExperiments.c_str());
		vector <float> value;
		for (int i=0; (results_of_experiments.good()) && (i < tmp.size()) ; i++)
		{
			std::getline (results_of_experiments, tmp_value);
			if(tmp_value.empty () || tmp_value.at (0) == '#') // Skip blank lines or comments
			continue;
			value.push_back(atof(tmp_value.c_str())+tmp.at(i));
		    // ROS_INFO("tmp_value.c_str() = %s",tmp_value.c_str() );
		    // ROS_INFO("atof -> tmp_value.c_str() = %f",atof(tmp_value.c_str()) );
		    // ROS_INFO("tmp[%i] = %f",i, tmp.at(i) );
		    // ROS_INFO("value[%i]= %f",i, value.at(i) );			
		}
		results_of_experiments.close();

		// 	ROS_INFO("number_of_exist_experiments = %i",number_of_exist_experiments );

		std::ofstream sum_results_of_experiments;
		sum_results_of_experiments.open (sumResultsOfExperiments.c_str(), std::ofstream::out);
		sum_results_of_experiments.precision(4);
		if (value.size() == 5 )
			sum_results_of_experiments <<value.at(0) <<"\n"<< value.at(1) <<"\n"<< value.at(2) <<"\n"<< value.at(3)<< "\n"<< value.at(4);

		sum_results_of_experiments.close();
    } 

return 0;	
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int average_all_experiments_results ( int total_number_of_experiments,
				   					  string name_of_approach)
{
    std::string sumResultsOfExperiments;
    sumResultsOfExperiments = ros::package::getPath("rug_simulated_user")+ "/result/sum_all_results_of_"+name_of_approach+"_experiments.txt";
	// sumResultsOfExperiments = /*ros::package::getPath("rug_simulated_user")+*/ "rug_simulated_user/result/sum_all_results_of_"+name_of_approach+"_experiments.txt";

    // ROS_INFO("results of expriments file path = %s",sumResultsOfExperiments.c_str() );
    
    if (!fexists(sumResultsOfExperiments.c_str()))
    {
		ROS_INFO("Expriments summary file of not exist");	
    }
	else
    {
		// ROS_INFO("File exist");
		string tmp_value;
		std::ifstream read_results_of_experiments;
		read_results_of_experiments.open (sumResultsOfExperiments.c_str());
		vector <float> value;
		for (int i=0; read_results_of_experiments.good(); i++)
		{
			std::getline (read_results_of_experiments, tmp_value);
			if(tmp_value.empty () || tmp_value.at (0) == '#') // Skip blank lines or comments
			continue;
			value.push_back(atof(tmp_value.c_str())/total_number_of_experiments);	    
		}
		read_results_of_experiments.close();

		std::string resultsOfExperiments;
		resultsOfExperiments = ros::package::getPath("rug_simulated_user")+ "/result/results_of_"+name_of_approach+"_experiments.txt";
		std::ofstream results_of_experiments;
		results_of_experiments.open (resultsOfExperiments.c_str(), std::ofstream::app);
		results_of_experiments.precision(4);
		results_of_experiments << "\n"<<"Avg."<<"\t"<<value.at(0) <<"\t\t"<< value.at(1) <<"\t\t"<< value.at(2) <<"\t\t"<< value.at(3)<< "\t\t"<< value.at(4);
		results_of_experiments << "\n---------------------------------------------------------------------------";
		results_of_experiments.close();
    }
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int reportAllExperimentalResults (int TP, int FP, int FN,int obj_num,			    
				    vector <float> average_class_precision,
				    float number_of_stored_instances, 
				    int number_of_taught_categories,
				    string name_of_approach
				   )
{
    //ROS_INFO("TEST report_all_experiments_results fucntion");
    int total_number_of_experiments;//TODO: shold be define as a paramere
	ros::param::get("/total_number_of_experiments", total_number_of_experiments);
	//ROS_INFO ("\t\t[-] ******* TOTAL EXPS = %d",total_number_of_experiments);

    unsigned int number_of_exist_experiments = 0;
    
    float average_class_precision_value = 0;
    for (int i = 0; i < average_class_precision.size(); i++)
    {
		average_class_precision_value += average_class_precision.at(i); 	    
    }
    average_class_precision_value = average_class_precision_value / average_class_precision.size();
	
    std::string resultsOfExperiments;
    resultsOfExperiments = ros::package::getPath("rug_simulated_user")+ "/result/results_of_"+name_of_approach+"_experiments.txt";

    int exp_num =1;
    double Success_Precision = 0;
    if (!fexists(resultsOfExperiments.c_str()))
    {
		// ROS_INFO("File not exist");
		Success_Precision = TP/double (TP+FP);
		double Recall = TP/double (TP+FN);

		std::ofstream results_of_experiments;
		results_of_experiments.open (resultsOfExperiments.c_str(), std::ofstream::out);
		results_of_experiments.precision(4);
		results_of_experiments << "\nNum"<<"\tIterations" <<"\t"<< "Categories" <<"\t"<< "Instances"<< "\t"<< "GS" << "\t\t"<< "ACS";
		results_of_experiments << "\n---------------------------------------------------------------------------";
		results_of_experiments << "\n"<<exp_num<<"\t"<<obj_num <<"\t\t"<< number_of_taught_categories <<"\t\t"<< number_of_stored_instances/(float)number_of_taught_categories <<"\t\t"<< Success_Precision<< "\t\t"<< average_class_precision_value;
		results_of_experiments << "\n---------------------------------------------------------------------------";
		results_of_experiments.close();
		results_of_experiments.clear();
    }
	else
    {
		// ROS_INFO("File exist");
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
		// ROS_INFO("number_of_exist_experiments = %i",number_of_exist_experiments );
		exp_num = (number_of_exist_experiments)/2;
		ROS_INFO("\t\t[-] number_of_exist_experiments = %i",exp_num );

		Success_Precision = TP/double (TP+FP);
		double Recall = TP/double (TP+FN);
		std::ofstream results_of_experiments;
		results_of_experiments.open (resultsOfExperiments.c_str(), std::ofstream::app);
		results_of_experiments.precision(4);
		results_of_experiments << "\n"<<exp_num<<"\t"<<obj_num <<"\t\t"<< number_of_taught_categories <<"\t\t"<< number_of_stored_instances/(float)number_of_taught_categories <<"\t\t"<< Success_Precision<< "\t\t"<< average_class_precision_value;
		results_of_experiments << "\n---------------------------------------------------------------------------";
		results_of_experiments.close();
    } 

	string tmp_path = "rug_simulated_user/result/results_of_"+name_of_approach+"_experiments.txt";
	ROS_INFO("results of expriments file path = %s",tmp_path.c_str() );

    sum_all_experiments_results(obj_num,
								Success_Precision,			    
								average_class_precision_value,
								number_of_stored_instances, 
								number_of_taught_categories,
								name_of_approach );

    if (exp_num == total_number_of_experiments)
    {
		average_all_experiments_results (total_number_of_experiments, name_of_approach);
    }
    
return 0;	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int reportAllExperimentalResultsGOODPlusColor (int TP, int FP, int FN, 
											int obj_num,			    
											vector <float> average_class_precision,
											float number_of_stored_instances, 
											int number_of_taught_categories,
											string name_of_approach,
											string color_space, 
											float weight_color, 
											int number_of_bins )
{
    //ROS_INFO("TEST report_all_experiments_results fucntion");
    int total_number_of_experiments;//TODO: shold be define as a paramere
	ros::param::get("/total_number_of_experiments", total_number_of_experiments);
	//ROS_INFO ("\t\t[-] ******* TOTAL EXPS = %d",total_number_of_experiments);

    unsigned int number_of_exist_experiments = 0;
    
    float average_class_precision_value = 0;
    for (int i = 0; i < average_class_precision.size(); i++)
    {
		average_class_precision_value += average_class_precision.at(i); 	    
    }
    average_class_precision_value = average_class_precision_value / average_class_precision.size();
	
    std::string resultsOfExperiments;
    resultsOfExperiments = ros::package::getPath("rug_simulated_user")+ "/result/results_of_"+name_of_approach+"_experiments.txt";

    int exp_num =1;
    double Success_Precision = 0;
    if (!fexists(resultsOfExperiments.c_str()))
    {
		// ROS_INFO("File not exist");
		Success_Precision = TP/double (TP+FP);
		double Recall = TP/double (TP+FN);

		std::ofstream results_of_experiments;
		results_of_experiments.open (resultsOfExperiments.c_str(), std::ofstream::out);
		results_of_experiments.precision(4);
		results_of_experiments << "\nNum"<<"\t#bins"<<"\tCSpace"<<"\tW"<<"\t\tIterations" <<"\t"<< "Categories" <<"\t"<< "Instances"<< "\t"<< "GS" << "\t\t"<< "ACS";
		results_of_experiments << "\n-----------------------------------------------------------------------------------------------------------------------";
		results_of_experiments << "\n"<<exp_num<<"\t"<<number_of_bins<<"\t"<<color_space<<"\t"<<weight_color<<"\t\t"<<obj_num <<"\t\t"<< number_of_taught_categories <<"\t\t"<< number_of_stored_instances/(float)number_of_taught_categories <<"\t\t"<< Success_Precision<< "\t\t"<< average_class_precision_value;
		results_of_experiments << "\n-----------------------------------------------------------------------------------------------------------------------";
		results_of_experiments.close();
		results_of_experiments.clear();
    }
	else
    {
		// ROS_INFO("File exist");
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
		// ROS_INFO("number_of_exist_experiments = %i",number_of_exist_experiments );
		exp_num = (number_of_exist_experiments)/2;
		ROS_INFO("\t\t[-] number_of_exist_experiments = %i",exp_num );

		Success_Precision = TP/double (TP+FP);
		double Recall = TP/double (TP+FN);
		std::ofstream results_of_experiments;
		results_of_experiments.open (resultsOfExperiments.c_str(), std::ofstream::app);
		results_of_experiments.precision(4);
		results_of_experiments << "\n"<<exp_num<<"\t"<<number_of_bins<<"\t"<<color_space<<"\t"<<weight_color<<"\t\t"<<obj_num <<"\t\t"<< number_of_taught_categories <<"\t\t"<< number_of_stored_instances/(float)number_of_taught_categories <<"\t\t"<< Success_Precision<< "\t\t"<< average_class_precision_value;
		results_of_experiments << "\n-----------------------------------------------------------------------------------------------------------------------";
		results_of_experiments.close();
    } 

	ROS_INFO("results of expriments file path = %s",resultsOfExperiments.c_str() );

    // sum_all_experiments_results ( obj_num,
	// 			  Success_Precision,			    
	// 			  average_class_precision_value,
	// 			  number_of_stored_instances, 
	// 			  number_of_taught_categories,
	// 			  name_of_approach );

    // if (exp_num == total_number_of_experiments)
    // {
	// 	average_all_experiments_results (total_number_of_experiments, name_of_approach);
    // }
    
return 0;	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int read_a_number_from_file ( string pakage_name,
			       string file_name)
{
    std::string path;
    path = ros::package::getPath(pakage_name.c_str())+ "/result/"+file_name;
    ROS_INFO("file path = %s",path.c_str() );
    string tmp_value;
    std::ifstream fnumber;
    int number =0;
    fnumber.open (path.c_str());
    for (int i=0; (fnumber.good()); i++)
    {
	std::getline (fnumber, tmp_value);
	if(tmp_value.empty () || tmp_value.at (0) == '#') // Skip blank lines or comments
	    continue;
	number =  atoi(tmp_value.c_str());
    }
    fnumber.close();

return number;	
    
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int write_a_number_to_file ( string pakage_name,
			       string file_name,
			       int number )
{
    std::string path;
    path = ros::package::getPath(pakage_name.c_str())+ "/result/"+file_name;
    ROS_INFO("file path = %s",path.c_str() );
    std::ofstream fnumber;
    fnumber.open (path.c_str(), std::ofstream::trunc);
    fnumber.precision(4);
    fnumber << number;
    fnumber.close();

	return 0;	
    
}

