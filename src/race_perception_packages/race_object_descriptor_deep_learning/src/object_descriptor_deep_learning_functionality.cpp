#ifndef _OBJECT_DESCRIPTOR_DEEP_LEARNING_CPP_
#define _OBJECT_DESCRIPTOR_DEEP_LEARNING_CPP_

//includes
#include <object_descriptor_deep_learning/object_descriptor_deep_learning_functionality.h>
#include <CGAL/Plane_3.h>
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>


//system includes
#include <stdio.h>
#include <stdlib.h>

//ROS includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>   
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/cloud_viewer.h>

//package includes
#include <race_perception_msgs/perception_msgs.h>
#include <race_perception_msgs/CompleteRTOV.h>

//Eigen includes
#include <Eigen/Core>

//PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <CGAL/Plane_3.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

////new added
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/pcd_io.h>
//new includes from preprocessing
//ROS includes
#include <ros/ros.h>
//#include "pcl_ros/impl/transforms.hpp"
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
//#include <pcl/common/impl/transforms.hpp>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//#include "/opt/ros/fuerte/stacks/geometry/tf_conversions/include/tf_conversions/tf_eigen.h"
#include <visualization_msgs/Marker.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <race_perception_utils/print.h>
#include <pcl_conversions/pcl_conversions.h>

//raceua includes
#include <race_perception_db/perception_db.h>
#include <race_perception_db/perception_db_serializer.h>
#include <race_perception_db/MsgTest.h>
#include <race_perception_utils/cycle.h>
#include <race_perception_utils/print.h>

#include <race_perception_msgs/TOVI.h>

#include <colormap_utils/colormap_utils.h>



using namespace std;
using namespace pcl;
using namespace ros;



// PointCloudPtrT cloud_reference;
// PointCloudPtrT initial_cloud_ref;


/* _________________________________
   |                                 |
   |        Library functions        |
   |_________________________________| */

int keypoint_selection( boost::shared_ptr<pcl::PointCloud<PointT> > target_pc, 
			float uniform_sampling_size,
			boost::shared_ptr<pcl::PointCloud<PointT> > uniform_keypoints,
			boost::shared_ptr<pcl::PointCloud<int> > uniform_sampling_indices
  		    )
{

    boost::shared_ptr<PointCloud<PointT> > cloud_filtered (new PointCloud<PointT>);
    pcl::VoxelGrid<PointT > voxelized_point_cloud;	
    voxelized_point_cloud.setInputCloud (target_pc);
    voxelized_point_cloud.setLeafSize (uniform_sampling_size, uniform_sampling_size, uniform_sampling_size);
    voxelized_point_cloud.filter (*cloud_filtered);

    //pcl::PointCloud<int> uniform_sampling_indices;
    for (int i =0; i < cloud_filtered->points.size() ;i++)
    {
	int nearest_point_index = 0;
	double minimum_distance = 1000;
	for (int j=0; j < target_pc->points.size(); j++)
	{		
	    double distance = sqrt( pow((cloud_filtered->points[i].x - target_pc->points[j].x) , 2) +
				    pow((cloud_filtered->points[i].y - target_pc->points[j].y) , 2) +
				    pow((cloud_filtered->points[i].z - target_pc->points[j].z), 2));
	    if (distance < minimum_distance)
	    {
		    nearest_point_index = j;
		    minimum_distance = distance;
	    }
	}
	uniform_sampling_indices->push_back(nearest_point_index);
    }

    pcl::copyPointCloud (*target_pc, uniform_sampling_indices->points, *uniform_keypoints);
    return 1;
}
   
int estimateSpinImages(boost::shared_ptr<PointCloud<PointT> > cloud, 
		float downsampling_voxel_size, 
		float normal_estimation_radius,
		int spin_image_width,
		float spin_image_cos_angle,
		size_t spin_image_minimum_neighbor_density,
		float spin_image_support_lenght,
		boost::shared_ptr< vector <SITOV> > spin_images_msg,
		size_t subsampled_spin_image_num_keypoints
		)
{ 
	const size_t spin_image_size = (spin_image_width+1) * (2*spin_image_width+1); //TODO put here documentation url where the formula is...
	
	//ROS_INFO("spin_image_size = %ld",spin_image_size);

	if ((spin_image_size != 153))
	{
	    if (spin_image_size != 91)
	    {
		if (spin_image_size != 45)
		{
		    ROS_ERROR("- Incorrect width. Cannot use.");		
		    return 0;
		}
	    }
	}


	typedef Histogram<153> SpinImage; // TODO find a way to use a variable size histogram. define new type for spin image.

	//STEP 1: Downsample the input point cloud using downsampling voxel size
	PointCloud<PointT>::Ptr downsampled_pc (new PointCloud<PointT>);
	PointCloud<int> sampled_indices;
	UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud (cloud);
	uniform_sampling.setRadiusSearch (downsampling_voxel_size/*0.01f*/);
	//uniform_sampling.compute (sampled_indices);////PCL 1.7;
	//copyPointCloud (*cloud, sampled_indices.points, *downsampled_pc);////PCL 1.7;

	uniform_sampling.filter (*downsampled_pc);//PCL 1.8;
	
	//STEP 2: Compute normals for downsampled point cloud
	search::KdTree<PointT>::Ptr kdtree (new search::KdTree<PointT>);
	NormalEstimation<PointT, Normal> normal_estimation;
	normal_estimation.setInputCloud (downsampled_pc);
	normal_estimation.setSearchMethod (kdtree);
	PointCloud<Normal>::Ptr downsampled_pc_with_normals (new PointCloud< Normal>);
	normal_estimation.setRadiusSearch ( normal_estimation_radius/*0.05*/);
	PointCloud<Normal>::Ptr Keypoints_with_normal (new PointCloud< Normal>);
	normal_estimation.compute (*Keypoints_with_normal);

	//STEP 3: Estimate the spin image for the downsampled_pc with the downsampled_pc_with_normals
	SpinImageEstimation<PointT, Normal, SpinImage > spin_image_descriptor(spin_image_width, spin_image_cos_angle, spin_image_minimum_neighbor_density);
	spin_image_descriptor.setInputCloud (downsampled_pc);
	spin_image_descriptor.setInputNormals (Keypoints_with_normal); 
	spin_image_descriptor.setSearchMethod (kdtree); //Use the same KdTree from the normal estimation
	PointCloud<SpinImage>::Ptr spin_images (new PointCloud<SpinImage>);
	spin_image_descriptor.setRadiusSearch (spin_image_support_lenght); //support_lengh (base= radius and lenght = 2*radius)
	spin_image_descriptor.compute (*spin_images); //Actually compute the spin images

	for (size_t i = 0; i < spin_images->size(); i++) // remove NAN values from the computed spin image (replace them by 0)  
	    {
		for (size_t j = 0; j < spin_image_size; j++)
		{
			if(std::isnan(spin_images->points.at(i).histogram[j]))
			{
				spin_images->points.at(i).histogram[j]=0;
			}
		}
	}

	size_t subsample_step;
	if (subsampled_spin_image_num_keypoints != 0)
	{
	    subsample_step = (spin_images->size())/subsampled_spin_image_num_keypoints; //to compute the subsampling step;
	}
	else if (subsampled_spin_image_num_keypoints==0)
	{
		subsample_step = 1;	
	}
	else
	{
	    if (subsample_step < 1)
        {
            ROS_INFO("Spin image function: Error computed a subsample_step =%ld < 1. This cannot be correct. Here subsampling is cancled. spin_images->size()=%ld subsampled_spin_image_num_keypoints=%ld", subsample_step, spin_images->size(), subsampled_spin_image_num_keypoints);
		    subsample_step = 1;	
        }
    }

    if (subsample_step == 0)
		subsample_step = 1;	

//#if _SPIN_IMAGE_DEBUG_
	//ROS_INFO("Spin image function: subsample_step = %ld",subsample_step);
//#endif

	//STEP 4: Copy the spin image to the message format spin_images_msg
	spin_images_msg->erase(spin_images_msg->begin(), spin_images_msg->end()); //to erase all spin image 
	//ROS_INFO("\t\t[-]-number of keypoints : %ld (spin-images)", spin_images->size());

	for (size_t i = 0; i < spin_images->size(); i += subsample_step) // remove NAN values from the computed spin image (replace them by 0)  
	{
	    SITOV tmp;
	    for (size_t j = 0; j < spin_image_size; j++)
	    {
		tmp.spin_image.push_back( spin_images->points.at( i ).histogram[ j ] );
	    }
	    spin_images_msg->push_back( tmp );	
	}

	// Free the allocated pointers
	downsampled_pc.reset();
	kdtree.reset();
	downsampled_pc_with_normals.reset(); //added by miguel
	Keypoints_with_normal.reset();
	spin_images.reset();//added by miguel
	return 1;
}

int estimateSpinImages2(boost::shared_ptr<PointCloud<PointT> > cloud, 
		float uniform_downsampling_voxel_size, 
		float normal_estimation_radius,
		int spin_image_width,
		float spin_image_cos_angle,
		size_t spin_image_minimum_neighbor_density,
		float spin_image_support_lenght,
		boost::shared_ptr< vector <SITOV> > spin_images_msg,
		boost::shared_ptr<pcl::PointCloud<int> > uniform_sampling_indices)
{ 
	const size_t spin_image_size = (spin_image_width+1) * (2*spin_image_width+1); //TODO put here documentation url where the formula is...
	
	//ROS_INFO("spin_image_size = %ld",spin_image_size);

	if ((spin_image_size != 153))
	{
	    if (spin_image_size != 91)
	    {
		if (spin_image_size != 45)
		{
		    ROS_ERROR("- Incorrect width. Cannot use.");		
		    return 0;
		}
	    }
	}

	typedef Histogram<153> SpinImage; // TODO find a way to use a variable size histogram. define new type for spin image.

	//STEP 1: Compute normals for downsampled point cloud
	search::KdTree<PointT>::Ptr kdtree (new search::KdTree<PointT>);
	NormalEstimation<PointT, Normal> normal_estimation;
	normal_estimation.setInputCloud (cloud);
	normal_estimation.setSearchMethod (kdtree);
	PointCloud<Normal>::Ptr downsampled_pc_with_normals (new PointCloud< Normal>);
	normal_estimation.setRadiusSearch ( normal_estimation_radius/*0.05*/);
	PointCloud<Normal>::Ptr Keypoints_with_normal (new PointCloud< Normal>);
	normal_estimation.compute (*Keypoints_with_normal);

	//STEP 2: Estimate the spin image for the downsampled_pc with the downsampled_pc_with_normals
	SpinImageEstimation<PointT, Normal, SpinImage > spin_image_descriptor(spin_image_width, spin_image_cos_angle, spin_image_minimum_neighbor_density);
	spin_image_descriptor.setInputCloud (cloud);
	spin_image_descriptor.setInputNormals (Keypoints_with_normal); 
	spin_image_descriptor.setSearchMethod (kdtree); //Use the same KdTree from the normal estimation
	PointCloud<SpinImage>::Ptr spin_images (new PointCloud<SpinImage>);
	spin_image_descriptor.setRadiusSearch (spin_image_support_lenght); //support_lengh (base= radius and lenght = 2*radius)
	spin_image_descriptor.compute (*spin_images); //Actually compute the spin images

	//STEP 3: Copy the keypoints spin image to the message format spin_images_msg
	spin_images_msg->erase(spin_images_msg->begin(), spin_images_msg->end()); //to erase all spin image 
	for (int i =0; i < uniform_sampling_indices->size(); i++)
	{	
	    SITOV tmp;
	    for (size_t j = 0; j < spin_image_size; j++)
	    {
		tmp.spin_image.push_back( spin_images->points.at( uniform_sampling_indices->at(i) ).histogram[ j ] );
	    }
	    spin_images_msg->push_back( tmp );	
	}
	
	//ROS_INFO("Size of spin_image_msg %i",spin_images_msg->size() );
	// Free the allocated pointers
	kdtree.reset();
	downsampled_pc_with_normals.reset(); //added by miguel
	Keypoints_with_normal.reset();
	spin_images.reset();//added by miguel
	return 1;
}


int differenceBetweenSpinImage( SITOV  sp1, 
				SITOV  sp2,
				float &difference)	
{
	//ROS_INFO("\t\t[-]- Inside differenceBetweenSpinImage function");
	difference = 0;
	//ROS_INFO("\t\t[-]- sp1 size = %ld , sp2 size = %ld", sp1.spin_image.size(), sp2.spin_image.size());
	if (sp1.spin_image.size() != sp2.spin_image.size())
	{
		return 0;
	}

	for (size_t j = 0; j < sp1.spin_image.size(); j++)
	{
		difference += pow( sp1.spin_image.at(j) - sp2.spin_image.at(j), 2.0);
		//ROS_INFO("%ld diff: %f:", j, difference);
	}

	return 1;
}

int differenceBetweenObjectViews( std::vector< SITOV> spin_images1, 
				   std::vector< SITOV> spin_images2,
				  float &difference)
{
	
	//D(t,O)=(Sigma (min D(tl,Ok)))/q
	//q= is the size of spin images of the target object
	
	difference=0;
	int q = spin_images1.size();
	if (q == 0)
	{
	   ROS_ERROR("Error: Size of target object (q) is 0- could not compute distance");
	}
	
	//ROS_INFO("\t\t[1]-Object view 1 with %ld keypoints", spin_images1.size());
	//ROS_INFO("\t\t[2]-Object view 2 with %ld keypoints", spin_images2.size());
	//int index = -1;
	for (size_t i = 0; i < spin_images1.size(); i++)
	{  
	    float minimum_distance = 1000000;
	    for (size_t j = 0; j < spin_images2.size(); j++)
	    {
		float si_difference = 0;

		//ROS_INFO("[%ld, %ld] ",i,j);
		SITOV sp1;
		SITOV sp2;

		sp1 = spin_images1.at(i);
		sp2 = spin_images2.at(j);
		
		if ( !differenceBetweenSpinImage( sp1, sp2, si_difference) )		    
		{
		    ROS_ERROR("Error comparing spin images");
		    return 0;	
		}

		if (si_difference < minimum_distance)
		{
			minimum_distance = si_difference;
			//index = j;
		}

	    }

	    (difference) += minimum_distance;
	}

	difference = difference;
	//ROS_INFO("\t\t[-]-Difference: %f", difference);
	difference=difference/q;
	//ROS_INFO("\t\t[-]-q: %i",q);
	//ROS_INFO("\t\t[-]-Normalize difference: %f",*difference);
	return 1;
}//function

void printSpinImages(vector <SITOV> spin_images) 
{
	ROS_INFO("\t\t[-]-Object view with %ld spin images:",spin_images.size());

	for (size_t i = 0; i < spin_images.size(); i++) 
	{
		printf("key point %ld[\n", i);
		for (size_t j = 0; j < spin_images.at( i ).spin_image.size(); j++) 
		{
			printf("%0.5f; ", spin_images.at( i ).spin_image.at( j ));
		}
		printf("]\n");
	}
}


void  visulazeObjectViewSpinImageMatlab ( vector <SITOV> objectView,  const char* Filename)
{
   ofstream SpinImageMATLAB;
   SpinImageMATLAB.open (Filename); 
   SpinImageMATLAB << "close all;\nfigure;\n ";

    std:: string Object_name;
    std:: string Temp_Object_name = Filename;// for writing object name in result file;
    int rfind = Temp_Object_name.rfind("//")+2;       
    int len = Temp_Object_name.length();
    
    for (size_t i=0; i<(len-rfind); i++)
	Object_name += Temp_Object_name.at(i+rfind);
    
    string tmp = Object_name;
    Object_name.resize((tmp.size() - 2));
   
   for(size_t i = 0; i < objectView.size(); ++i)
    { 
	SpinImageMATLAB << "SI" << i << "=[";
	for (size_t j = 0; j < objectView.at( i ).spin_image.size()-1; j++)
	{		
		SpinImageMATLAB << objectView.at( i ).spin_image.at( j ) <<","; 
	}
	
	SpinImageMATLAB << objectView.at( i ).spin_image.at( objectView.at( i ).spin_image.size()-1 ) << "]; \n"; 
	
	
	SpinImageMATLAB << "subplot(6,6," << i+1 << ");\n";
	SpinImageMATLAB << "plot( SI" << i << ");\n";
	SpinImageMATLAB << "grid on;\n";
	SpinImageMATLAB << "axis([0 155 0 0.2]);\n";
        SpinImageMATLAB << "xlabel('" << Object_name.c_str() << "-SI" << i << "'); \n\n";

     }//for
   
    SpinImageMATLAB << "OVSI =[";
    for(size_t i = 0; i < objectView.size(); ++i)
    { 
		for (size_t j = 0; j < objectView.at( i ).spin_image.size()-1; j++)
	{		
		SpinImageMATLAB << objectView.at( i ).spin_image.at( j ) <<","; 
	}
	
	SpinImageMATLAB << objectView.at( i ).spin_image.at( objectView.at( i ).spin_image.size()-1 ) << "; \n"; 
	
    }//for
    SpinImageMATLAB << "];\n";
    SpinImageMATLAB << "figure;\n";
    SpinImageMATLAB << "plot( OVSI);\n";

    SpinImageMATLAB.close();
    SpinImageMATLAB.clear();
    std::cout<<"\n\t - Visualization: M File Created .... \n";
}



int estimateVFH(boost::shared_ptr<PointCloud<PointT> > cloud, 
		float downsampling_voxel_size, 
		float normal_estimation_radius,
		boost::shared_ptr< vector <SITOV> > viewpoint_feature_histogram_msg,
		size_t subsampled_viewpoint_feature_histogram_num_keypoints
		)
{ 
	const size_t VFH_size = 308;	
	ROS_INFO("VFH_size = %ld",VFH_size);

	//STEP 1: Downsample the input point cloud using downsampling voxel size
	PointCloud<PointT>::Ptr downsampled_pc (new PointCloud<PointT>);
	PointCloud<int> sampled_indices;
	UniformSampling<PointT> uniform_sampling;
	uniform_sampling.setInputCloud (cloud);
	uniform_sampling.setRadiusSearch (downsampling_voxel_size/*0.01f*/);
	//uniform_sampling.compute (sampled_indices);////PCL 1.7;
	//copyPointCloud (*cloud, sampled_indices.points, *downsampled_pc);////PCL 1.7;

	uniform_sampling.filter (*downsampled_pc);//PCL 1.8;

	//STEP 2: Compute normals for downsampled point cloud
	search::KdTree<PointT>::Ptr kdtree (new search::KdTree<PointT>);
	NormalEstimation<PointT, Normal> normal_estimation;
	normal_estimation.setInputCloud (downsampled_pc);
	normal_estimation.setSearchMethod (kdtree);
	PointCloud<Normal>::Ptr downsampled_pc_with_normals (new PointCloud< Normal>);
	normal_estimation.setRadiusSearch ( normal_estimation_radius/*0.05*/);
	PointCloud<Normal>::Ptr Keypoints_with_normal (new PointCloud< Normal>);
	normal_estimation.compute (*Keypoints_with_normal);

	//STEP 3: Estimate the VFH for the downsampled_pc with the downsampled_pc_with_normals
	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (downsampled_pc);
	vfh.setInputNormals (Keypoints_with_normal);
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	vfh.setSearchMethod (tree);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
	vfh.compute (*vfhs);
		
// 	pcl::visualization::PCLHistogramVisualizer histogram; 
// 	histogram.addFeatureHistogram (*vfhs, 308, "VFH", 640, 200); 
// 	histogram.spin ();
// 	
	
	for (size_t i = 0; i < vfhs->size(); i++) // remove NAN values from the computed spin image (replace them by 0)  
	{
		for (size_t j = 0; j < VFH_size; j++)
		{
			if(std::isnan(vfhs->points.at(i).histogram[j]))
			{
				vfhs->points.at(i).histogram[j]=0;
			}
		}
	}

	size_t subsample_step;
	if (subsampled_viewpoint_feature_histogram_num_keypoints != 0)
	{
	    subsample_step = (vfhs->size())/subsampled_viewpoint_feature_histogram_num_keypoints; //to compute the subsampling step;
	}
	else
	{
	    subsample_step = 1;	
	}	
	
	if (subsample_step < 1)
        {
		ROS_INFO("Spin image function: Error computed a subsample_step =%ld < 1. This cannot be correct. Here subsampling is cancled. VFHs->size()=%ld subsampled_viewpoint_feature_histogram_num_keypoints=%ld", 
			 subsample_step, vfhs->size(), subsampled_viewpoint_feature_histogram_num_keypoints);
		subsample_step = 1;	
        }
    

//#if _SPIN_IMAGE_DEBUG_
	ROS_INFO("Spin image function: subsample_step = %ld",subsample_step);
//#endif
	//STEP 4: Copy the spin image to the message format viewpoint_feature_histogram_msg
	viewpoint_feature_histogram_msg->erase(viewpoint_feature_histogram_msg->begin(), viewpoint_feature_histogram_msg->end()); //to erase all spin image 
	//ROS_INFO("\t\t[-]-number of keypoints : %ld (spin-images)", spin_images->size());

	for (size_t i = 0; i < vfhs->size(); i+=subsample_step)
	{
	    SITOV tmp;
	    for (size_t j = 0; j < VFH_size; j++)
	    {
		tmp.spin_image.push_back( vfhs->points.at( i ).histogram[ j ] );
	    }
	    viewpoint_feature_histogram_msg->push_back( tmp );	
	}

	// Free the allocated pointers
	downsampled_pc.reset();
	kdtree.reset();
	downsampled_pc_with_normals.reset(); //added by miguel
	Keypoints_with_normal.reset();
	vfhs.reset();//added by miguel
	return 1;
}



  
// 	    bool signDisambiguationFlag = false;
// 	    int sign = 1;
// 	    int threshold = 10;	 
// boost::shared_ptr<tf::TransformListener> _p_transform_listener;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int compute_bounding_box_dimensions(boost::shared_ptr<PointCloud<PointT> > pc, geometry_msgs::Vector3& dimensions)
{
	PointT minimum_pt;
	PointT maximum_pt;

	getMinMax3D(*pc, minimum_pt, maximum_pt); // min max for bounding box
	dimensions.x = (maximum_pt.x - minimum_pt.x); 
	dimensions.y = (maximum_pt.y - minimum_pt.y); 
	dimensions.z = (maximum_pt.z - minimum_pt.z); 
	
	return 1;
}


// template <typename T>
int project_pc_to_plane(boost::shared_ptr<pcl::PointCloud<T> > pc_in, boost::shared_ptr<pcl::ModelCoefficients> coefficients, boost::shared_ptr<pcl::PointCloud<T> > pc_out)
{
	//Create the projection object
	pcl::ProjectInliers<T> projection;
	projection.setModelType(pcl::SACMODEL_NORMAL_PLANE); //set model type
	projection.setInputCloud(pc_in);
	projection.setModelCoefficients(coefficients);
	projection.filter(*pc_out);

	return 1;
}

void print_tf_transform(tf::Transform* tf, std::string s)
{
	std::ostringstream ss;

	//s.append("fixed_frame_id=" + tf->frame_id_ + " frame_id="+ tf->child_frame_id_);

	tf::Quaternion q(tf->getRotation().x(),tf->getRotation().y(),tf->getRotation().z(),tf->getRotation().w());
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	ROS_INFO("(roll, pitch, yaw) = (%f, %f, %f)",roll, pitch, yaw);


	ss << std::fixed << std::setprecision(5) << tf->getOrigin().x();
	s.append("\nOrigin x=" + ss.str());
	ss.str("");

	ss << std::fixed << std::setprecision(5) << tf->getOrigin().y();
	s.append(" y=" + ss.str());
	ss.str("");

	ss << std::fixed << std::setprecision(5) << tf->getOrigin().z();
	s.append(" z=" + ss.str());
	ss.str("");

	ss << std::fixed << std::setprecision(5) << tf->getRotation().x();
	s.append("\nRotation x=" + ss.str());
	ss.str("");

	ss << std::fixed << std::setprecision(5) << tf->getRotation().y();
	s.append(" y=" + ss.str());
	ss.str("");

	ss << std::fixed << std::setprecision(5) << tf->getRotation().z();
	s.append(" z=" + ss.str());
	ss.str("");

	ss << std::fixed << std::setprecision(5) << tf->getRotation().w();
	s.append(" w=" + ss.str());
	ss.str("");

	ROS_INFO("%s",s.c_str());
}

// template <typename T>
void printObjectViewHistograms ( vector < vector <T> > object_view_histograms)
{

  ROS_INFO("\t\t[-]- given object_view_histograms has %ld histogram:",object_view_histograms.size());
  for (size_t i = 0; i < object_view_histograms.size(); i++) 
    {
	printf("Plane %ld = [", i);
	for (size_t j = 0; j < object_view_histograms.at( i ).size(); j++) 
	{
		printf("%d; ", object_view_histograms.at( i ).at( j ));
	}
	printf("]\n");
    }
}

// template <typename T>
void printHistogram ( vector <T>  histogram, string plane_name)
{

  ROS_INFO("\t\t[-]- given histograms has %ld bins:",histogram.size());
  printf("\nPlane %s = [", plane_name.c_str());
  for (size_t i = 0; i < histogram.size(); i++) 
  {
    cout<< histogram.at(i)<<", " ;
  }
  printf("]\n");
  
}

void convert2DhistogramTo1Dhisftogram(vector <vector <int> >  _2DHistogram, vector <int>  &histogram)
{
  
  ROS_INFO("\t\t[-]- given 2D histograms has [%ld,%ld] dimensions.", _2DHistogram.size(), _2DHistogram.at(0).size());
  
  for (int i=0 ; i < _2DHistogram.size(); i++)
  {
   
    for (int j=0; j < _2DHistogram.at(i).size(); j++)
    {
      //ROS_INFO("\n _2DHistogram.at(i).at(j)= %d", _2DHistogram.at(i).at(j) );
      histogram.push_back(_2DHistogram.at(i).at(j));
    }
  }
}

void convert2DhistogramTo1Dhisftogram(vector <vector <float> >  _2DHistogram, vector <float>  &histogram)
{
  
  ROS_INFO("\t\t[-]- given 2D histograms has [%ld,%ld] dimensions.", _2DHistogram.size(), _2DHistogram.at(0).size());
  
  for (int i=0 ; i < _2DHistogram.size(); i++)
  {
   
    for (int j=0; j < _2DHistogram.at(i).size(); j++)
    {
      //ROS_INFO("\n _2DHistogram.at(i).at(j)= %d", _2DHistogram.at(i).at(j) );
      histogram.push_back(_2DHistogram.at(i).at(j));
    }
  }
}


// 	    template <typename T>
int XYsignDisambiguation(boost::shared_ptr<pcl::PointCloud<T> >  XOY_projected_view,
		      int threshold, int &sign )
{

  int Xpositive =0; 
  int Xnegative =0; 

  int Ypositive =0; 
  int Ynegative =0; 

  //XOY page
  for (int i=0; i< XOY_projected_view -> points.size(); i++)
  {
	
	  if (XOY_projected_view->points.at(i).x >0.015)
	  {
	      Xpositive ++;
	  }
	  else if (XOY_projected_view->points.at(i).x < - 0.015)
	  {
	      Xnegative ++;
	  }
	  
	  if (XOY_projected_view->points.at(i).y >0.015)
	  {
	      //Ypositive ++;
	      Xpositive ++;
	  }
	  else if (XOY_projected_view->points.at(i).y < -0.015)
	  {
	      //Ynegative ++;
	      Xnegative ++;
	  }
  }
  
  
  if ((Xpositive < Xnegative) and (Xnegative - Xpositive > threshold))
  {
      sign = -1;
      ROS_INFO("axes directions must be filiped -> Xpositive = %d , Xnegative = %d, diffrence is = %d", Xpositive, Xnegative, abs(Xpositive - Xnegative));

  }
  else 
  {
      sign = 1;
      ROS_INFO("axes directions are correctly defined->  Xpositive = %d , Xnegative = %d, diffrence is = %d", Xpositive, Xnegative, abs(Xpositive - Xnegative));
  }
  
  if ((Ypositive < Ynegative) and (Ynegative - Ypositive > threshold))
  {
      sign = -1;
      ROS_INFO("directions must be filiped -> diffrence is = %d", abs(Xpositive - Xnegative));

  }
  else 
  {
      sign = 1;
      ROS_INFO("directions are correctly defined-> diffrence is = %d", abs(Xpositive - Xnegative));
  }	
  return 0;
}

// 	    template <typename T>
int XsignDisambiguation(boost::shared_ptr<pcl::PointCloud<T> >  XoZ_projected_view,
		      int threshold, int &sign )
{

  int Xpositive =0; 
  int Xnegative =0; 

  //XoZ page
  for (int i=0; i< XoZ_projected_view -> points.size(); i++)
  {
	
	  if (XoZ_projected_view->points.at(i).x >0.015)
	  {
	      Xpositive ++;
	  }
	  else if (XoZ_projected_view->points.at(i).x < - 0.015)
	  {
	      Xnegative ++;
	  }
// 			else//debug
// 			{
// 			  ROS_INFO(" X = %f", XoZ_projected_view->points.at(i).x );
// 			}
	  
  }
  
  if ((Xpositive < Xnegative) and (Xnegative - Xpositive >= threshold))
  {
      sign = -1;
      ROS_INFO("Direction of X axis must be filiped -> Xpositive = %d , Xnegative = %d, diffrence is = %d", Xpositive, Xnegative, abs(Xpositive - Xnegative));

  }
  else 
  {
      sign = 1;
      ROS_INFO("Direction of X axis is correctly defined->  Xpositive = %d , Xnegative = %d, diffrence is = %d", Xpositive, Xnegative, abs(Xpositive - Xnegative));
  }
		  
  return 0;
}


// 	    template <typename T>
int YsignDisambiguation(boost::shared_ptr<pcl::PointCloud<T> >  YoZ_projected_view,
		      int threshold, int &sign )
{

  int Ypositive =0; 
  int Ynegative =0; 

  //YoZ page
  for (int i=0; i< YoZ_projected_view -> points.size(); i++)
  {
	
	  if (YoZ_projected_view->points.at(i).y >0.015)
	  {
	      Ypositive ++;
	  }
	  else if (YoZ_projected_view->points.at(i).y < - 0.015)
	  {
	      Ynegative ++;
	  }
// 			else//debug
// 			{
// 			  ROS_INFO(" Y = %f", YoZ_projected_view->points.at(i).y );
// 			}
  }
  
  if ((Ypositive < Ynegative) and (Ynegative - Ypositive >= threshold))
  {
      sign = -1;
      ROS_INFO("Direction of Y axis must be filiped -> Ypositive = %d , Ynegative = %d, diffrence is = %d", Ypositive, Ynegative, abs(Ypositive - Ynegative));

  }
  else 
  {
      sign = 1;
      ROS_INFO("Direction of Y axis is correctly defined->  Ypositive = %d , Ynegative = %d, diffrence is = %d", Ypositive, Ynegative, abs(Ypositive - Ynegative));
  }
		  
  return 0;
}
  
      
      
// 	    template <typename T>
int YOZ2DObjectHistogram( boost::shared_ptr<pcl::PointCloud<T> >  YOZ_projected_view,
			double largest_side, 
			int number_of_bins, 
			int sign,
			vector < vector<int> > &YOZ_histogram)

{

  ROS_INFO("number_of_bins = %d , sign = %d", number_of_bins, sign);

  for (int i =0; i < number_of_bins; i++)
  {
      vector<int> row;
      for (int j =0; j < number_of_bins; j++)
      {		
	  row.push_back(0);
      }
      YOZ_histogram.push_back(row);
  }

  ROS_INFO("YOZ_histogram has [%d , %d] bins", YOZ_histogram.size(), YOZ_histogram.at(0).size());
  ROS_INFO("sign = %d",sign);

  double x = largest_side/2; 
  double y = largest_side/2; 
  double z = largest_side/2; 
  double interval_x = largest_side/number_of_bins; 
  double interval_y = largest_side/number_of_bins; 
  double interval_z = largest_side/number_of_bins; 
  for (int i=0; i < YOZ_projected_view-> points.size(); i++)
  {
	geometry_msgs::Point p;
	//p.x = all_projected_views.at(view_i)->points.at(i).x + x;
	p.y = sign * YOZ_projected_view->points.at(i).y +  y;
	p.z = YOZ_projected_view ->points.at(i).z + z;
	
	//if adaptive_support_lenght parameter == false, some points might be projected outside of the plane, we must discard them.
	if ((trunc(p.y / interval_y) < YOZ_histogram.size()) and (trunc(p.z / interval_z) < YOZ_histogram.at(0).size())
	      and (trunc(p.y / interval_y) >= 0) and (trunc(p.z / interval_z) >= 0))
	{
	  YOZ_histogram.at(trunc(p.y / interval_y)).at(trunc(p.z / interval_z))++;
	}
	else 
	{
	  //debug
	  //ROS_INFO("YOZ: P(y).(%i)= %f --- dy= %f, interval_y = %f", i, p.y, y, interval_y );
	  //ROS_INFO("YOZ: P(z).(%i)= %f --- dz= %f, interval_z = %f", i, p.z, z, interval_z );
	  //ROS_INFO("(trunc(p.y / interval_y)= %f and (trunc(p.z / interval_z) =%f", trunc(p.y / interval_y), trunc(p.z / interval_z));
	}
    }
    return 0;
}


// 	    template <typename T>
int XOZ2DObjectHistogram( boost::shared_ptr<pcl::PointCloud<T> >  XOZ_projected_view,
			double largest_side, 
			int number_of_bins,
			int sign,
			vector < vector<int> > &XOZ_histogram)

{
  
  for (int i =0; i<number_of_bins; i++)
  {
      vector<int> row;
      for (int j =0; j< number_of_bins; j++)
      {		
	  row.push_back(0);
      }
      XOZ_histogram.push_back(row);
  }
  
  double x = largest_side/2; 
  double y = largest_side/2; 
  double z = largest_side/2; 
  double interval_x = largest_side/number_of_bins; 
  double interval_y = largest_side/number_of_bins; 
  double interval_z = largest_side/number_of_bins; 
  for (int i=0; i < XOZ_projected_view-> points.size(); i++)
  {
	geometry_msgs::Point p;
	p.x =sign *  XOZ_projected_view->points.at(i).x +  x;
	//p.y = XOZ_projected_view->points.at(i).y + y;
	p.z = XOZ_projected_view ->points.at(i).z + z;
	//ROS_INFO("YOZ: P(x).(%i)= %f --- dx= %f", i, XOZ_projected_view->points.at(i).x, x );
	//ROS_INFO("(trunc(p.x / interval_x)= %f and (trunc(p.z / interval_z) =%f", trunc(p.x / interval_x), trunc(p.z / interval_z));
	
	//if adaptive_support_lenght parameter == false, some points might be projected outside of the plane, we must discard them.
	if ((trunc(p.x / interval_x) < XOZ_histogram.size()) and (trunc(p.z / interval_z) < XOZ_histogram.at(0).size())
	      and (trunc(p.x / interval_x) >=0) and (trunc(p.z / interval_z) >=0))
	{
	    XOZ_histogram.at(trunc(p.x / interval_x)).at(trunc(p.z / interval_z))++;
	}
	else 
	{
	  //debug
	  //ROS_INFO("XOZ: P(y).(%i)= %f --- dy= %f, interval_y = %f", i, p.x, x, interval_x );
	  //ROS_INFO("XOZ: P(z).(%i)= %f --- dz= %f, interval_z = %f", i, p.z, z, interval_z );
	  //ROS_INFO("(trunc(p.x / interval_x)= %f and (trunc(p.z / interval_z) =%f", trunc(p.x / interval_x), trunc(p.z / interval_z));
	}
    }
    return 0;
}


// 	    template <typename T>
int XOY2DObjectHistogram( boost::shared_ptr<pcl::PointCloud<T> >  XOY_projected_view,
			double largest_side, 
			int number_of_bins, 
			int sign,
			vector < vector<int> > &XOY_histogram)

{


  for (int i =0; i<number_of_bins; i++)
  {
      vector<int> row;
      for (int j =0; j< number_of_bins; j++)
      {		
	  row.push_back(0);
      }
      XOY_histogram.push_back(row);
  }
  
  double x = largest_side/2; 
  double y = largest_side/2; 
  double z = largest_side/2; 
  double interval_x = largest_side/number_of_bins; 
  double interval_y = largest_side/number_of_bins; 
  double interval_z = largest_side/number_of_bins; 
  for (int i=0; i < XOY_projected_view-> points.size(); i++)
  {
	geometry_msgs::Point p;
	p.x = sign * XOY_projected_view->points.at(i).x +  x;
	p.y = sign * XOY_projected_view->points.at(i).y +  y;
	//p.z = XOY_projected_view ->points.at(i).z + z;
	//ROS_INFO("YOZ: P(x).(%i)= %f --- dx= %f", i, XOY_projected_view->points.at(i).x, x );
	//ROS_INFO("(trunc(p.x / interval_x)= %f and (trunc(p.y / interval_y) =%f", trunc(p.x / interval_x), trunc(p.y / interval_y));
	
	//if adaptive_support_lenght parameter == false, some points might be projected outside of the plane, we must discard them.
	if ((trunc(p.x / interval_x) < XOY_histogram.size()) and (trunc(p.y / interval_y) < XOY_histogram.at(0).size())and 
	      (trunc(p.x / interval_x) >= 0) and (trunc(p.y / interval_y) >=0))
	{
	    XOY_histogram.at(trunc(p.x / interval_x)).at(trunc(p.y / interval_y))++;
	}
	else 
	{
	  //debug
	  //ROS_INFO("XOY: P(x).(%i)= %f --- dx= %f, interval_x = %f", i, p.x, x, interval_x );
	  //ROS_INFO("XOY: P(y).(%i)= %f --- dy= %f, interval_y = %f", i, p.y, y, interval_y );
	  //ROS_INFO("(trunc(p.x / interval_x)= %f and (trunc(p.z / interval_z) =%f", trunc(p.x / interval_x), trunc(p.y / interval_y));
	}
    }
    return 0;
}
  
	
int normalizingHistogram(vector <int> histogram, 
			vector <float> &normalized_histogram)
{
  //compute sumation of all histogram's bins .
  int sum_all_bins = 0;    
  for (size_t i = 0; i < histogram.size(); i++)
  {
    sum_all_bins += histogram.at(i);
  }
  ROS_INFO("sum_all_bins = %d ", sum_all_bins);
  
  //normalizing histogram.
  float normalizing_bin = 0;
  for (size_t i = 0; i < histogram.size(); i++)
  {
      normalizing_bin = 0;
      if (sum_all_bins != 0)
      {  
	normalizing_bin =  histogram.at(i) / float (sum_all_bins);
	//cout <<  normalizing_bin << ", ";
      }
      normalized_histogram.push_back( normalizing_bin);
  }
  return 0;
}

int viewpointEntropy(vector <float> normalized_histogram,
		  float &entropy)
{
  //http://stats.stackexchange.com/questions/66108/why-is-entropy-maximised-when-the-probability-distribution-is-uniform
  //compute entropy (latex notation) : H(\textbf{m})= -\sum_{i=1}^{n^2} \textbf{m}_i~\log_2 ~\textbf{m}_i
  entropy =0;
  for (size_t i = 0; i < normalized_histogram.size(); i++)
  {

    if (normalized_histogram.at(i) != 0)
    {
      float entropy_tmp = normalized_histogram.at(i) * log2(normalized_histogram.at(i));
      //float entropy_tmp = (normalized_histogram.at(i)/normalized_histogram.size())* log2(normalized_histogram.at(i)/normalized_histogram.size());
      entropy = entropy + entropy_tmp;
      //ROS_INFO("entropy_tmp = %f", entropy_tmp);
    }
  }
  entropy = -entropy;
  ROS_INFO("entropy = %f", entropy);

  return 0;
}

int viewpointEntropyNotNormalized(vector <int> not_normalized_histogram,
		  float &entropy)
{

  //TODO : the name of this fuction should be "viewPointEntropy" and the other one is a just entropy 
  //compute entropy (latex notation) : H(\textbf{m})= -\sum_{i=1}^{n^2} \textbf{m}_i~\log_2 ~\textbf{m}_i
  entropy =0;
  for (size_t i = 0; i < not_normalized_histogram.size(); i++)
  {

    //if (not_normalized_histogram.at(i) > 0)
    if (not_normalized_histogram.at(i) != 0)
    {
      //float entropy_tmp = not_normalized_histogram.at(i) * log2(not_normalized_histogram.at(i));
      float entropy_tmp = float(1/*not_normalized_histogram.at(i)*//float(not_normalized_histogram.size()))
				*log2(float(/*not_normalized_histogram.at(i)*/1)/float(not_normalized_histogram.size()));
      entropy = entropy + entropy_tmp;
      //entropy ++;
      //ROS_INFO("entropy_tmp = %f", entropy_tmp);
    }
  }
  entropy = -entropy;
  //entropy = entropy/float(not_normalized_histogram.size());
  ROS_INFO("entropy = %f", entropy);

  return 0;
}

int findMaxViewPointsEntropy( vector <float> view_point_entropy, 
			    int &index)
{
index = 0;
for (int i=1; i < view_point_entropy.size(); i++ )
{
    if (view_point_entropy.at(i) > view_point_entropy.at(index))
    {
      index = i;
    }
}

return 0;
}
  
int kullbackLiebler ( vector <float>  Ptheta,
		    vector <float>  Qtheta,
		    float &similarity )   
{ 
  similarity = 0;
  float Distance_P_Q =0;
  float Distance_Q_P =0;
  
  for (int i = 0; i< Ptheta.size(); i++)
  {
      if ((Ptheta.at(i)!=0) and (Qtheta.at(i) != 0))
      {
	  //Distance_P_Q += Ptheta.at(i)*(log2(Ptheta.at(i)/Qtheta.at(i)));
	  Distance_P_Q +=  Ptheta.at(i)*(log2(Ptheta.at(i)/Qtheta.at(i)));
	  Distance_Q_P +=  Qtheta.at(i)* (log2(Qtheta.at(i)/Ptheta.at(i)));
// // 			Distance_P_Q +=  float(Ptheta.at(i))/float(Ptheta.size()) *(log2(Ptheta.at(i)/Qtheta.at(i)));
// 			Distance_Q_P +=  float(Qtheta.at(i))/float(Ptheta.size()) * (log2(Qtheta.at(i)/Ptheta.at(i)));
      }

  }	
  
  //similarity = pow (Distance_P_Q,2) ;
  //similarity = Distance_P_Q ;
  similarity = 0.5 * (Distance_P_Q + Distance_Q_P);
  ROS_INFO("D(P, Q) = %f", similarity);
  //likelihood = similarity;

return 0;
}

int avrageHistograms( vector< float> histogram1,
		      vector< float> historam2,
		      vector< float> historam3,
		      vector< float> &average)
{
  for (int i=0; i <histogram1.size(); i++ )
  {
    average.push_back(float(histogram1.at(i)+historam2.at(i)+historam3.at(i))/float(3.00));
  }
  return 0;
}


int meanOfHistogram(vector< float> histogram, float &mean)
{ 	    
  // http://www.stat.yale.edu/Courses/1997-98/101/rvmnvar.htm
  float mu = 0;
  for (int i = 0; i < histogram.size(); i++)
  {
      mu += (i+1)*histogram.at(i);
  }
  mean = mu;
  return 0;
}
int varianceOfHistogram(vector< float> histogram, float mean, float &variance)
{
  //https://people.richland.edu/james/lecture/m170/ch06-prb.html
  // http://www.stat.yale.edu/Courses/1997-98/101/rvmnvar.htm
  
  float variance_tmp = 0;
  for (int i = 0; i < histogram.size(); i++)
  {
      variance_tmp += pow((i+1)-mean,2)*histogram.at(i);
  }
  variance = variance_tmp;
  return 0;
}
int subtractTowHistogram(vector< float> histogram1,
		      vector< float> historam2,
		      vector< float> &subtract)
{

  for (int i=0; i <histogram1.size(); i++ )
  {
    if (histogram1.at(i) > historam2.at(i))
    {
	subtract.push_back(histogram1.at(i)-historam2.at(i));
    }
    else
    {
	  subtract.push_back(0);
    }
  }
  
  return 0;
}





int objectViewHistogram( int maximum_entropy_index,
		      vector <float> view_point_entropy,
		      vector< vector<float> >normalized_projected_views,
		      vector< float> &sorted_normalized_projected_views,
		      string &std_name_of_sorted_projected_plane /*debug*/
		    )
{
  float disTmp1 = 0;
  float disTmp2 = 0;
  
  float disTmpAvg1 = 0;
  float disTmpAvg2 = 0;
  
  float disTmpRef1 = 0;
  float disTmpRef2 = 0;

  float disTmpEnt1 = 0;
  float disTmpEnt2 = 0;
  float wRef=50, wAvg=25; 
  
  float variance1 = 0;
  float variance2 = 0;
  float mean =0;
  std::setprecision(3);
  vector <float> subtract0;
  vector <float> subtract1;
  vector <float> subtract2;
  vector <float> average;

  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
					      normalized_projected_views.at(maximum_entropy_index).begin(), 
					      normalized_projected_views.at(maximum_entropy_index).end());    

  avrageHistograms(normalized_projected_views.at(0),normalized_projected_views.at(1),normalized_projected_views.at(2),average);
  subtractTowHistogram(normalized_projected_views.at(0),average, subtract0); 
  subtractTowHistogram(normalized_projected_views.at(1),average, subtract1);
  subtractTowHistogram(normalized_projected_views.at(2),average, subtract2); 
    
  switch (maximum_entropy_index)
  {
    case 0 :
      
      std_name_of_sorted_projected_plane += "YoZ - ";
      
      meanOfHistogram(normalized_projected_views.at(1), mean);
      varianceOfHistogram(normalized_projected_views.at(1), mean, variance1);
      meanOfHistogram(normalized_projected_views.at(2), mean);
      varianceOfHistogram(normalized_projected_views.at(2), mean, variance2);
      disTmp1=variance1;
      disTmp2=variance2;
      
      if (disTmp1 <= disTmp2)
      {
	std_name_of_sorted_projected_plane += "XoZ (" + boost::lexical_cast<std::string>(disTmp1)+ 
					    ") - XoY ("+ boost::lexical_cast<std::string>(disTmp2)+ ")";
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(1).begin(),
						  normalized_projected_views.at(1).end());
	
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(2).begin(),
						  normalized_projected_views.at(2).end());
      }
      else
      {		     
	std_name_of_sorted_projected_plane += "XoY (" + boost::lexical_cast<std::string>(disTmp2)+ 
					    ") - XoZ ("+ boost::lexical_cast<std::string>(disTmp1)+ ")";
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(2).begin(),
						  normalized_projected_views.at(2).end());
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(1).begin(),
						  normalized_projected_views.at(1).end());
      }
      break;
      
    case 1 :
      std_name_of_sorted_projected_plane += "XoZ - ";
      
      meanOfHistogram(normalized_projected_views.at(0), mean);
      varianceOfHistogram(normalized_projected_views.at(0), mean, variance1);
//       meanOfHistogram(normalized_projected_views.at(1), mean);
//       varianceOfHistogram(normalized_projected_views.at(1), mean, variance2);
      
      meanOfHistogram(normalized_projected_views.at(2), mean);
      varianceOfHistogram(normalized_projected_views.at(2), mean, variance2);
      
      
      disTmp1=variance1;
      disTmp2=variance2;
      
      
      if (disTmp1 <= disTmp2)
      {
	std_name_of_sorted_projected_plane += "YoZ (" + boost::lexical_cast<std::string>(disTmp1)+ 
					    ") - XoY ("+ boost::lexical_cast<std::string>(disTmp2)+ ")";
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(0).begin(),
						  normalized_projected_views.at(0).end());
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(2).begin(),
						  normalized_projected_views.at(2).end());
      }
      else
      {
	  std_name_of_sorted_projected_plane += "XoY (" + boost::lexical_cast<std::string>(disTmp2)+ 
					    ") - YoZ ("+ boost::lexical_cast<std::string>(disTmp1)+ ")";
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(2).begin(),
						  normalized_projected_views.at(2).end());
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(0).begin(),
						  normalized_projected_views.at(0).end());
      }
      break;
	
    case 2 :
      std_name_of_sorted_projected_plane += "XoY - ";
		      
      meanOfHistogram(normalized_projected_views.at(0), mean);
      varianceOfHistogram(normalized_projected_views.at(0), mean, variance1);
      meanOfHistogram(normalized_projected_views.at(1), mean);
      varianceOfHistogram(normalized_projected_views.at(1), mean, variance2);
      disTmp1=variance1;
      disTmp2=variance2;
      
      
      if (disTmp1 <= disTmp2)
      {
	  std_name_of_sorted_projected_plane += "YoZ (" + boost::lexical_cast<std::string>(disTmp1)+ 
					    ") - XoZ ("+ boost::lexical_cast<std::string>(disTmp2)+ ")";
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(0).begin(),
						  normalized_projected_views.at(0).end());
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(1).begin(),
						  normalized_projected_views.at(1).end());
      }
      else
      {
	  std_name_of_sorted_projected_plane += "XoZ (" + boost::lexical_cast<std::string>(disTmp1)+ 
					    ") - YoZ ("+ boost::lexical_cast<std::string>(disTmp2)+ ")";
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(1).begin(),
						  normalized_projected_views.at(1).end());
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(0).begin(),
						  normalized_projected_views.at(0).end());
      }
      break;  
      
    default:
      ROS_INFO("The maximum_entropy_index is out of scope [0-2], It is %d", maximum_entropy_index);
      break;
  }
  
return 0;
}


int normalized2Dhistogram (vector< vector< int > > not_normalized_2D_projection, vector< vector< float > > &normalized_2Dprojection, int object_size)
{
    cout << "normalizedMatrix = \n";
    for (size_t i = 0; i < not_normalized_2D_projection.size(); ++i)
    {
      vector <float> tmp; 
      for (size_t j = 0; j < not_normalized_2D_projection.size(); ++j)
      {
	    tmp.push_back(not_normalized_2D_projection.at(i).at(j) / float (object_size));
	    cout << tmp.at(j)<<",";
      }
      normalized_2Dprojection.push_back(tmp);
      cout <<"\n";
    }
    return 0;   
}


int compute_2D_Variance ( vector <vector <float> > projection, float &variance_projection)
{
  
  ROS_INFO("INSIDE compute_2D_Variance");
  ROS_INFO("size of projected view [%d, %d]", projection.size(), projection.at(0).size());

  float mi=0, mj=0;
  
  for (int i=0; i < projection.size(); i++)
  {
    for(int j=0; j< projection.size(); j++)
    {
      mi += i* projection.at(i).at(j);
      mj += j* projection.at(i).at(j);
    }
  }
  
  float mean_i = mi/(pow (projection.size(),2));
  float mean_j = mj/(pow (projection.size(),2));

  ROS_INFO("[mean_i, mean_j] of the given projection= [%f, %f]", mean_i , mean_j);

  
  float si=0, sj=0;
  for (int i=0; i < projection.size(); i++)
  {
    for(int j=0; j< projection.size(); j++)
    {
      //variance along x axis
      si += pow((i-mean_i),2) * projection.at(i).at(j);;
      //variance along y axis
      sj += pow((j-mean_j),2) * projection.at(i).at(j);;
    }
  }
  variance_projection = (sqrt(si)+ sqrt(sj)) / 2.0;//Standarad diviation
  ROS_INFO("2D variance of the given projection= %f", variance_projection);
  return 0;
}


int compute_average_distance ( vector <vector <float> > projection, float &weighted_variance)
{
  //http://math.stackexchange.com/questions/819747/measuring-variance-of-2d-data-points
  
  ROS_INFO("INSIDE compute_average_distance");
  ROS_INFO("size of projected view [%d, %d]", projection.size(), projection.at(0).size());

  float mi=0, mj=0, sw2=0;
  
  for (int i=0; i < projection.size(); i++)
  {
    for(int j=0; j< projection.size(); j++)
    {
	mi += i * projection.at(i).at(j);
	mj += j * projection.at(i).at(j);
	sw2 = pow (projection.at(i).at(j), 2);
    }
  }
  
  float mean_i = mi/(pow (projection.size(),2));
  float mean_j = mj/(pow (projection.size(),2));

  ROS_INFO("[mean_i, mean_j] of the given projection= [%f, %f]", mean_i , mean_j);

  
  float wv=0;
  for (int i=0; i < projection.size(); i++)
  {
    for(int j=0; j< projection.size(); j++)
    {
       wv += (pow((i-mean_i) , 2) + pow((i-mean_i) , 2)) * projection.at(i).at(j);
    }
  }
  weighted_variance = wv;
  ROS_INFO("weighted variance of the given projection = %f", wv);
  return 0;
}




int objectViewHistogramUsing2Dvariance( int maximum_entropy_index,
					  vector <float> view_point_with_maximum_entropy,
					  vector < vector<float> > normalized_projected_views /*all projection_views[0,1,2]*/,
					  vector < vector < vector<float> > > normalized_2D_projected_views,
					  vector< float> &sorted_normalized_projected_views,
					  int sorting_criterion,
					  string &std_name_of_sorted_projected_plane /*debug*/
					)
{
  std::setprecision(3);
  
  float disTmp1 = 0;
  float disTmp2 = 0; 
  float variance1 = 0;
  float variance2 = 0;
 
  ROS_INFO("size of projected view [%d, %d]", normalized_2D_projected_views.at(0).size(), normalized_2D_projected_views.at(0).at(0).size());
  ROS_INFO("size of projected view [%d, %d]", normalized_2D_projected_views.at(1).size(), normalized_2D_projected_views.at(1).at(1).size());
  ROS_INFO("size of projected view [%d, %d]", normalized_2D_projected_views.at(2).size(), normalized_2D_projected_views.at(2).at(2).size());
  
  
  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
					      view_point_with_maximum_entropy.begin(), view_point_with_maximum_entropy.end());    

  switch (maximum_entropy_index)
  {
    case 0 :
      
      std_name_of_sorted_projected_plane += "YoZ - ";
     

      
      if (sorting_criterion == SORT_BY_ENTROPY_AND_2D_VARIANCE_SUM)
      {
	compute_average_distance (normalized_2D_projected_views.at(1), variance1);
	compute_average_distance (normalized_2D_projected_views.at(2), variance2);
      }
      else
      {
	compute_2D_Variance ( normalized_2D_projected_views.at(1), variance1);
	compute_2D_Variance ( normalized_2D_projected_views.at(2), variance2);		
      }
      
      disTmp1=variance1;
      disTmp2=variance2;
      
      if (disTmp1 <= disTmp2)
      {
	std_name_of_sorted_projected_plane += "XoZ (" + boost::lexical_cast<std::string>(disTmp1)+ 
					    ") - XoY ("+ boost::lexical_cast<std::string>(disTmp2)+ ")";
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(1).begin(), normalized_projected_views.at(1).end());
	
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(2).begin(), normalized_projected_views.at(2).end());
      }
      else
      {		     
	std_name_of_sorted_projected_plane += "XoY (" + boost::lexical_cast<std::string>(disTmp2)+ 
					    ") - XoZ ("+ boost::lexical_cast<std::string>(disTmp1)+ ")";
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						   normalized_projected_views.at(2).begin(), normalized_projected_views.at(2).end());
	sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(1).begin(), normalized_projected_views.at(1).end());
      }
      break;
      
    case 1 :
      std_name_of_sorted_projected_plane += "XoZ - ";

      if (sorting_criterion == SORT_BY_ENTROPY_AND_2D_VARIANCE_SUM)
      {
	  compute_average_distance (normalized_2D_projected_views.at(0), variance1);
	  compute_average_distance (normalized_2D_projected_views.at(2), variance2);

      }
      else 
      {
	  compute_2D_Variance ( normalized_2D_projected_views.at(0), variance1);	
	  compute_2D_Variance ( normalized_2D_projected_views.at(2), variance2);
      }
      disTmp1=variance1;
      disTmp2=variance2;
      
      
      if (disTmp1 <= disTmp2)
      {
	  std_name_of_sorted_projected_plane += "YoZ (" + boost::lexical_cast<std::string>(disTmp1)+ 
					      ") - XoY ("+ boost::lexical_cast<std::string>(disTmp2)+ ")";
	  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						    normalized_projected_views.at(0).begin(), normalized_projected_views.at(0).end());
	  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						    normalized_projected_views.at(2).begin(), normalized_projected_views.at(2).end());
      }
      else
      {
	  std_name_of_sorted_projected_plane += "XoY (" + boost::lexical_cast<std::string>(disTmp2)+ 
					    ") - YoZ ("+ boost::lexical_cast<std::string>(disTmp1)+ ")";
	  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						    normalized_projected_views.at(2).begin(), normalized_projected_views.at(2).end());
	  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						  normalized_projected_views.at(0).begin(), normalized_projected_views.at(0).end());
      }
      break;
	
    case 2 :
      std_name_of_sorted_projected_plane += "XoY - "; 
      
      
      if (sorting_criterion == SORT_BY_ENTROPY_AND_2D_VARIANCE_SUM)
      {
	  compute_average_distance (normalized_2D_projected_views.at(0), variance1);
	  compute_average_distance (normalized_2D_projected_views.at(1), variance2);

      }
      else 
      {
	  compute_2D_Variance ( normalized_2D_projected_views.at(0), variance1);	
	  compute_2D_Variance ( normalized_2D_projected_views.at(1), variance2);
      }
      
      
      disTmp1=variance1;
      disTmp2=variance2;
      
      if (disTmp1 <= disTmp2)
      {
	  std_name_of_sorted_projected_plane += "YoZ (" + boost::lexical_cast<std::string>(disTmp1)+ 
					    ") - XoZ ("+ boost::lexical_cast<std::string>(disTmp2)+ ")";
	  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						    normalized_projected_views.at(0).begin(), normalized_projected_views.at(0).end());
	  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						    normalized_projected_views.at(1).begin(), normalized_projected_views.at(1).end());
      }
      else
      {
	  std_name_of_sorted_projected_plane += "XoZ (" + boost::lexical_cast<std::string>(disTmp1)+ 
					    ") - YoZ ("+ boost::lexical_cast<std::string>(disTmp2)+ ")";
	  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						    normalized_projected_views.at(1).begin(), normalized_projected_views.at(1).end());
	  sorted_normalized_projected_views.insert ( sorted_normalized_projected_views.end(), 
						    normalized_projected_views.at(0).begin(), normalized_projected_views.at(0).end());
      }
      break;  
      
    default:
      ROS_INFO("The maximum_entropy_index is out of scope [0-2], It is %d", maximum_entropy_index);
      break;
  }
  
return 0;
}




int compute_largest_side_of_bounding_box(geometry_msgs::Vector3 dimensions,  double &largest_side )
{
  
      if ((dimensions.y >= dimensions.x))
      {
	if (dimensions.y >= dimensions.z)
	{
	  largest_side = dimensions.y;
	}
	else if (dimensions.z >= dimensions.x)
	{
	  largest_side= dimensions.z;
	}
	
      } 
      else if(dimensions.z >= dimensions.x)
      {
	if (dimensions.z >= dimensions.y)
	{
	  largest_side = dimensions.z;
	}
      }
      else
      {
	largest_side = dimensions.x;
      }

      largest_side += 0.02;

      return 0;
}

int downSampling ( boost::shared_ptr<PointCloud<PointT> > cloud, 		
		  float downsampling_voxel_size, 
		  boost::shared_ptr<PointCloud<PointT> > downsampled_pc)
{
  
	// Downsample the input point cloud using downsampling voxel size	
	 
	// Create the filtering object
  	VoxelGrid<PointT> voxel_grid_downsampled_pc;
	voxel_grid_downsampled_pc.setInputCloud (cloud);
	voxel_grid_downsampled_pc.setLeafSize (downsampling_voxel_size, downsampling_voxel_size, downsampling_voxel_size);
	voxel_grid_downsampled_pc.filter (*downsampled_pc);

	
  return 0;
}

int computeDistanceBetweenProjections (vector <vector <float> > projection1, vector <vector <float> > projection2, float &distance)
{
  float sum  = 0 ;
  for (size_t i =0; i < projection1.size(); i++)
  { for (size_t j =0; j < projection1.size(); j++)
    {
      float d = projection1.at(i).at(j) - projection2.at(i).at(j);
      sum += pow(d,2);
    }
  } 
  distance = sqrt(sum);
  return 0;  
}

int projectionDistinctivness (vector <vector <float> > projection1, vector <vector <float> > projection2, vector <vector <float> > projection3,
			      float &distinctivness)
{
   float d0 =0, d1 =0;
   computeDistanceBetweenProjections(projection1,projection2, d0);
   computeDistanceBetweenProjections(projection1,projection3, d1);
   distinctivness = d0 + d1;
//   distinctivness = computeDistanceBetweenProjections(projection1,projection2) + computeDistanceBetweenProjections(projection1,projection3);
  return 0;
}



int objectViewHistogramUsingDistinctiveness( vector < vector <vector <float> > > normalized_projected_views,
					  vector <float > & object_description,
					  string &std_name_of_sorted_projected_plane )
{
  float distinctivness0=0, distinctivness1 =0, distinctivness2 =0;
  projectionDistinctivness (normalized_projected_views.at(0),normalized_projected_views.at(1),normalized_projected_views.at(2), distinctivness0);
  projectionDistinctivness (normalized_projected_views.at(1),normalized_projected_views.at(0),normalized_projected_views.at(2), distinctivness1);
  projectionDistinctivness (normalized_projected_views.at(2),normalized_projected_views.at(0),normalized_projected_views.at(1), distinctivness2);

  vector <float> v0, v1, v2;
  convert2DhistogramTo1Dhisftogram(normalized_projected_views.at(0), v0);
  convert2DhistogramTo1Dhisftogram(normalized_projected_views.at(1), v1);
  convert2DhistogramTo1Dhisftogram(normalized_projected_views.at(2), v2);
 
  if (distinctivness0 > distinctivness1 && distinctivness0 > distinctivness2)
  {
      object_description.insert(object_description.end(), v0.begin(), v0.end());
      if (distinctivness1 > distinctivness2) {
	object_description.insert(object_description.end(), v1.begin(), v1.end());
 	object_description.insert(object_description.end(), v2.begin(), v2.end());
      }
      else {
	object_description.insert(object_description.end(), v2.begin(), v2.end());
 	object_description.insert(object_description.end(), v1.begin(), v1.end());
      }
    }
  else if (distinctivness1 > distinctivness0 && distinctivness1 > distinctivness2)
    {
      object_description.insert(object_description.end(), v1.begin(), v1.end());
      if (distinctivness0 > distinctivness2) {
	object_description.insert(object_description.end(), v0.begin(), v0.end());
 	object_description.insert(object_description.end(), v2.begin(), v2.end());
      }
      else {
	object_description.insert(object_description.end(), v2.begin(), v2.end());
 	object_description.insert(object_description.end(), v0.begin(), v0.end());
    }
  }
  else {
      object_description.insert(object_description.end(), v2.begin(), v2.end());
      if (distinctivness0 > distinctivness1) {
	object_description.insert(object_description.end(), v0.begin(), v0.end());
 	object_description.insert(object_description.end(), v1.begin(), v1.end());
      }
      else {
	object_description.insert(object_description.end(), v1.begin(), v1.end());
 	object_description.insert(object_description.end(), v0.begin(), v0.end());
    }
  }
  return 0;
}

int compuet_object_description( boost::shared_ptr<pcl::PointCloud<T> > target_pc,
				 int adaptive_support_lenght,
				 double global_image_width,
				 int threshold,
				 int number_of_bins,
				 boost::shared_ptr<pcl::PointCloud<T> > &pca_object_view,
				 Eigen::Vector3f &center_of_bbox,
				 vector < boost::shared_ptr<pcl::PointCloud<T> > > &vector_of_projected_views, 
				 double &largest_side, 
				 int &sign,
				 vector <float> &view_point_entropy,
				 string &std_name_of_sorted_projected_plane,
				 vector< float > &object_description )	
{

      
// 	float downsampling_voxel_size= 0.001;
// 	downSampling ( target_pc,  downsampling_voxel_size,  target_pc);
// 	
	ROS_INFO("//////////////////////////////// compuet_object_description ///////////////////////////////////////////");

	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_x (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_y (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_z (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	bool visualize = false;
	PointXYZ pt; 

	
	/* ____________________________
	  |                            |
	  | construct ORF based on PCA |
	  |____________________________| */
	
	  ///////////////////////////////////////////////// the theory of new shape descriptor ////////////////////////////////////////////////////////////////
	  // //NOTE  the PCA base reference frame construction basically does:
	  // 1) compute the centroid (c0, c1, c2) and the normalized covariance
	  // 2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
	  // 3) move the points in that RF --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
	  // 4) compute the max, the min and the center of the diagonal (mean_diag)
	  // 5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) 
	  //    the transformation you have to apply is Rotation = (e0, e1, e0 X e1) & Translation = Rotation * mean_diag + (c0, c1, c2)

	  
	  // compute principal directions
	  
	  Eigen::Vector4f centroid;
	  pcl::compute3DCentroid(*target_pc, centroid);
	  Eigen::Matrix3f covariance;
	  computeCovarianceMatrixNormalized(*target_pc, centroid, covariance);
	  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	  Eigen::Matrix3f eigVectors = eigen_solver.eigenvectors();
	  eigVectors.col(2) = eigVectors.col(0).cross(eigVectors.col(1));
	  std::cout << "eigen vectores :  \n " << eigVectors << std::endl;


// 	  std::cout << "eigen vectores \n " << eigVectors << std::endl;
// 	  std::cout << "column1: " << eigVectors.col(0) << std::endl;
// 	  std::cout << "column2: " << eigVectors.col(1) << std::endl;
// 	  std::cout << "column3: " << eigVectors.col(2) << std::endl;
	  
	  setprecision(20);
	  Eigen::Vector3f eigen_values =  eigen_solver.eigenvalues();
	  std::cout << "The eigenvalues of the covariance matrix before sorting are \n:" << eigen_values << endl;
// 	  std::cout << "The eigenvalue of the z axis is :" << eigen_values(0,0) << endl;
// 	  std::cout << "The eigenvalue of the Y axis is :" << eigen_values(0,1) << endl;
// 	  std::cout << "The eigenvalue of the x axis is :" << eigen_values(0,2) << endl;
// 	  ROS_INFO("eigenvalue(0) = %f \t eigenvalue(1) = %f \t eigenvalue(2) = %f", eigen_values(0,0),  eigen_values(0,1),  eigen_values(0,2) );
		  	  
// 	  std::sort(eigenValues.begin(), eigenValues.end());
//	  std::cout <<std::setprecision(4)<<"eigen vectores after sorting: " << eigenValues << endl;
	  
	  Eigen::Matrix3f eigVectors_tmp;
	  Eigen::Vector3f eigen_values_tmp;
// 	  //sorting eigen vectors based on eigen values
	  eigVectors.col(0)= eigVectors.col(2);
	  eigVectors.col(2) = eigVectors.col(0).cross(eigVectors.col(1));

	  std::cout << "eigen vectores cross product :  \n " << eigVectors << std::endl;

	  // 	  std::cout << "eigen vectores after sorting \n " << eigVectors << std::endl;
// 	  std::cout << "column1 after sort: " << eigVectors.col(0) << std::endl;
// 	  std::cout << "column2 after sort: " << eigVectors.col(1) << std::endl;
// 	  std::cout << "column3 after sort: " << eigVectors.col(2) << std::endl;

// 	  std::cout << "The eigenvalues of the covariance matrix after sorting are: \n" << eigen_values << endl;
// 	  std::cout << "The eigenvalue of the X axis is :" << eigen_values(0,0) << endl;
// 	  std::cout << "The eigenvalue of the Y axis is :" << eigen_values(0,1) << endl;
// 	  std::cout << "The eigenvalue of the Z axis is :" << eigen_values(0,2) << endl;
	  
	  

	  // move the points to the PCA based reference frame
	  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
	  p2w.block<3,3>(0,0) = eigVectors.transpose();
	  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
	  
	  //Declare a boost share ptr to the PCA_pointCloud
// 	  boost::shared_ptr<PointCloud<PointT> > pca_pc (new PointCloud<PointT>); 
		 
  
	  pcl::PointCloud<PointT> cPoints;
	  pcl::transformPointCloud(*target_pc, *pca_object_view, p2w);
	  //compute the max, the min and the center of the diagonal (mean_diag)
	  PointT min_pt, max_pt;
	  pcl::getMinMax3D(*pca_object_view, min_pt, max_pt);
	  const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
	  // centroid transform
	  Eigen::Quaternionf qfinal(eigVectors);//rotation matrix
	  center_of_bbox = eigVectors*mean_diag + centroid.head<3>(); // Translation = Rotation * center_diag + (c0, c1, c2)
	  std::cout << "center of box (x,y,z) =" << center_of_bbox << std::endl;

      /* _________________________________
	|                           	   |
	| construct three projection view |
	|_________________________________| */
			
	// ax+by+cz+d=0, where b=c=d=0, and a=1, or said differently, the YoZ plane.
	pcl::ModelCoefficients::Ptr coefficientsX (new pcl::ModelCoefficients ());
	coefficientsX->values.resize (4);
	coefficientsX->values[0] = 1.0; coefficientsX->values[1] = 0; coefficientsX->values[2] = 0; coefficientsX->values[3] = 0;
	project_pc_to_plane(pca_object_view, coefficientsX, initial_cloud_proj_x);
	for (int i=0; i< initial_cloud_proj_x->points.size(); i++)
	{
	    initial_cloud_proj_x->points.at(i).x = 0.3;
	}
	vector_of_projected_views.push_back(initial_cloud_proj_x);

	
	//ax+by+cz+d=0, where a=c=d=0, and b=1, or said differently, the XoZ plane.
	pcl::ModelCoefficients::Ptr coefficientsY (new pcl::ModelCoefficients ());
	coefficientsY->values.resize (4);
	coefficientsY->values[0] = 0.0; coefficientsY->values[1] = 1.0; coefficientsY->values[2] = 0; coefficientsY->values[3] = 0;

	project_pc_to_plane(pca_object_view, coefficientsY, initial_cloud_proj_y);
	for (int i=0; i< initial_cloud_proj_y->points.size(); i++)
	{
	    initial_cloud_proj_y->points.at(i).y = 0.3;
	}
	vector_of_projected_views.push_back(initial_cloud_proj_y);

		  
	// ax+by+cz+d=0, where a=b=d=0, and c=1, or said differently, the XoY plane.
	pcl::ModelCoefficients::Ptr coefficientsZ (new pcl::ModelCoefficients ());
	coefficientsZ->values.resize (4); 
	coefficientsZ->values[0] = 0; coefficientsZ->values[1] = 0; coefficientsZ->values[2] = 1.0;   coefficientsZ->values[3] = 0;
	project_pc_to_plane(pca_object_view, coefficientsZ, initial_cloud_proj_z);
	for (int i=0; i< initial_cloud_proj_z->points.size(); i++)
	{
	    initial_cloud_proj_z->points.at(i).z = 0.30;
	}		
	vector_of_projected_views.push_back(initial_cloud_proj_z);
      
	  
      /* ____________________________
	|                           |
	|  Axes sign disambiguation |
	|___________________________| */
			
	geometry_msgs::Vector3 dimensions;
	compute_bounding_box_dimensions(pca_object_view, dimensions);
	ROS_INFO("Box's dimensions (x, y, z) = (%f, %f, %f) ", dimensions.x, dimensions.y, dimensions.z);
	//double largest_side;
	compute_largest_side_of_bounding_box(dimensions ,largest_side);		

	if (adaptive_support_lenght == 0)
	{
	  ROS_INFO("Dimension of the (largest_side+0.02)/2 is %f m", largest_side/2);
	  ROS_INFO("adaptive_support_lenght = %d", adaptive_support_lenght);
	}
	else
	{
	  largest_side = global_image_width;
	  ROS_INFO("global_image_width = %f", largest_side);
	  ROS_INFO("adaptive_support_lenght = %d", adaptive_support_lenght);
	}
	  
      /* ____________________________
	|                           |
	|  Axes sign disambiguation |
	|___________________________| */
    
	int Sx=1, Sy=1;
	XsignDisambiguation(initial_cloud_proj_y, threshold, Sx );//XoZ Plane
	YsignDisambiguation(initial_cloud_proj_x, threshold, Sy );//YoZ Plane
	sign = Sx * Sy;
	ROS_INFO("sign = Sx (%d) * Sy(%d) = %d", Sx, Sy, sign);
	ROS_INFO("signDisambiguationFlag is updated");

	//eigVectors.col(2) = sign * eigVectors.col(2);
	
	// eigVectors.col(0)= Sx * eigVectors.col(0);
// 	 eigVectors.col(1)= Sy * eigVectors.col(1);
// 	 eigVectors.col(2) = eigVectors.col(0).cross(eigVectors.col(1));
// 	 
// 	 // move the points to the PCA based reference frame
// 	  //p2w(Eigen::Matrix4f::Identity());
// 	  p2w.block<3,3>(0,0) = eigVectors.transpose();
// 	  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
// 	  
// 	  pcl::transformPointCloud(*target_pc, *pca_object_view, p2w);
// 	  //compute the max, the min and the center of the diagonal (mean_diag)
// 	  pcl::getMinMax3D(*pca_object_view, min_pt, max_pt);
// 	  const Eigen::Vector3f mean_diag2 = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
// 	  // centroid transform
// // // 	  Eigen::Quaternionf qfinal(eigVectors);//rotation matrix
// // 	  center_of_bbox = eigVectors*mean_diag2 + centroid.head<3>(); // Translation = Rotation * center_diag + (c0, c1, c2)
// // 	  std::cout << "center of box (x,y,z) =" << center_of_bbox << std::endl;
// // // 


    /* _________________________________________________________
      |                       				       	  |
      |  compute histograms of projection of the given object   |
      |_________________________________________________________| */
	
	vector <int> complete_object_histogram;
	vector <int> complete_object_histogram_normalized;//each projection view is normalized sepreatly
	vector < vector<int> > XOZ_histogram;      
	vector < vector<int> > YOZ_histogram;
// 	vector <float> view_point_entropy;
	vector <vector <float> > normalized_projected_views;

	//projection along X axis
	YOZ2DObjectHistogram( initial_cloud_proj_x,
			      largest_side, 
			      number_of_bins, 
			      sign,
			      YOZ_histogram);
	
	vector <int> histogramYOZ1D;
	convert2DhistogramTo1Dhisftogram(YOZ_histogram, histogramYOZ1D);
	//printHistogram ( histogramYOZ1D, "YOZ");
	complete_object_histogram.insert(complete_object_histogram.end(), histogramYOZ1D.begin(), histogramYOZ1D.end());
	vector <float> normalized_histogramYoZ;
	normalizingHistogram( histogramYOZ1D, normalized_histogramYoZ);
	normalized_projected_views.push_back(normalized_histogramYoZ);
	
	//printHistogram ( normalized_histogramYoZ, "normalized YOZ");
	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), normalized_histogramYoZ.begin(), normalized_histogramYoZ.end());
	float YoZ_entropy = 0;
	//viewpointEntropy(normalized_histogramYoZ, YoZ_entropy);
	viewpointEntropyNotNormalized(histogramYOZ1D, YoZ_entropy);

	ROS_INFO("viewpointEntropyYoZ = %f", YoZ_entropy);
	view_point_entropy.push_back(YoZ_entropy);

	//projection along Y axis
	XOZ2DObjectHistogram( initial_cloud_proj_y,
			      largest_side, 
			      number_of_bins, 
			      sign,
			      XOZ_histogram);
      
	vector <int> histogramXOZ1D;
	convert2DhistogramTo1Dhisftogram(XOZ_histogram, histogramXOZ1D);
	//printHistogram ( histogramXOZ1D, "XOZ");
	complete_object_histogram.insert(complete_object_histogram.end(), histogramXOZ1D.begin(), histogramXOZ1D.end());

	vector <float> normalized_histogramXoZ;
	normalizingHistogram( histogramXOZ1D, normalized_histogramXoZ);
	//printHistogram ( normalized_histogramXoZ, "normalized XOZ");
	normalized_projected_views.push_back(normalized_histogramXoZ);

	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), 
						  normalized_histogramXoZ.begin(), 
						  normalized_histogramXoZ.end());
	float XoZ_entropy = 0;
	//viewpointEntropy(normalized_histogramXoZ, XoZ_entropy);
	viewpointEntropyNotNormalized(histogramXOZ1D, XoZ_entropy);
	ROS_INFO("viewpointEntropyXoZ = %f", XoZ_entropy);
	view_point_entropy.push_back(XoZ_entropy);


	//projection along Z axis
	vector < vector<int> > XOY_histogram;		
	XOY2DObjectHistogram( initial_cloud_proj_z,
				largest_side, 
				number_of_bins, 
				sign,
				XOY_histogram);

	vector <int> histogramXOY1D;
	convert2DhistogramTo1Dhisftogram(XOY_histogram, histogramXOY1D);
	//printHistogram ( histogramXOY1D, "XOY");		
	complete_object_histogram.insert(complete_object_histogram.end(), histogramXOY1D.begin(), histogramXOY1D.end());
	//printHistogram ( complete_object_histogram, "complete_object_histogram");
	
	vector <float> normalized_histogramXoY;
	normalizingHistogram( histogramXOY1D, normalized_histogramXoY);
	//printHistogram ( normalized_histogramXoY, "normalized XoY");
	normalized_projected_views.push_back(normalized_histogramXoY);

	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), normalized_histogramXoY.begin(), normalized_histogramXoY.end());
	float XoY_entropy = 0;
	//viewpointEntropy(normalized_histogramXoY, XoY_entropy);
	viewpointEntropyNotNormalized(histogramXOY1D, XoY_entropy);

	ROS_INFO("viewpointEntropyXoY = %f", XoY_entropy);
	view_point_entropy.push_back(XoY_entropy);

	//printHistogram ( complete_object_histogram_normalized, "complete_object_histogram_normalized");
	
	vector <float> normalized_histogram;
	normalizingHistogram( complete_object_histogram, normalized_histogram);
	//printHistogram ( normalized_histogram, "normalized_complete_object_histogram");

	int maximum_entropy_index = 0;
	findMaxViewPointsEntropy(view_point_entropy, maximum_entropy_index);
	ROS_INFO("Summary of entropy : \n H(YoZ) = %f, H(XoZ) = %f, H(XoY) = %f, Max_ind = %d", 
		  view_point_entropy.at(0), view_point_entropy.at(1), view_point_entropy.at(2) , maximum_entropy_index );		
	
// 	vector< float > object_description;
// 	string std_name_of_sorted_projected_plane;
	objectViewHistogram( maximum_entropy_index,
			    view_point_entropy,
			    normalized_projected_views,
			    object_description,
			    std_name_of_sorted_projected_plane
			  );
	
	ROS_INFO("Projected views are sorted as follows: %s", std_name_of_sorted_projected_plane.c_str());   		
	//printHistogram ( sorted_normalized_projected_views, "sorted_normalized_projected_views");
	

	return 0; 
}

int compuet_object_description_real_demo( boost::shared_ptr<pcl::PointCloud<T> > target_pc,
				 int adaptive_support_lenght,
				 double global_image_width,
				 int threshold,
				 int number_of_bins,
				 geometry_msgs::Vector3 dimensions,
				 vector < boost::shared_ptr<pcl::PointCloud<T> > > &vector_of_projected_views, 
				 double &largest_side, 
				 int &sign,
				 vector <float> &view_point_entropy,
				 string &std_name_of_sorted_projected_plane,
				 vector< float > &object_description )	
{

      
// 	float downsampling_voxel_size= 0.001;
// 	downSampling ( target_pc,  downsampling_voxel_size,  target_pc);
// 	
	ROS_INFO("//////////////////////////////// compuet_object_description_real_demo ///////////////////////////////////////////");
	ROS_INFO("//////////////////////////////// dimensions (%2.2f, %2.2f, %2.2f)  ///////////////////////////////////////////", 
		   dimensions.x , dimensions.y, dimensions.z	);
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_x (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_y (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_z (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	bool visualize = false;
	PointXYZ pt; 

	
	/* ____________________________
	  |                            |
	  | construct ORF based on PCA |
	  |____________________________| */
	
	  //IN online demo, all these parts have been done by object tracking

      /* _________________________________
	|                           	   |
	| construct three projection view |
	|_________________________________| */

	
      
	*initial_cloud_proj_x = *target_pc;
	for (int i=0; i< initial_cloud_proj_x->points.size(); i++)
	{
	    initial_cloud_proj_x->points.at(i).x = 0.3;
	}
	vector_of_projected_views.push_back(initial_cloud_proj_x);

	
	*initial_cloud_proj_y = *target_pc;
	for (int i=0; i< initial_cloud_proj_y->points.size(); i++)
	{
	    initial_cloud_proj_y->points.at(i).y = 0.3;
	}
	vector_of_projected_views.push_back(initial_cloud_proj_y);

		  
	*initial_cloud_proj_z = *target_pc;
	for (int i=0; i< initial_cloud_proj_z->points.size(); i++)
	{
	    initial_cloud_proj_z->points.at(i).z = 0.30;
	}		
	vector_of_projected_views.push_back(initial_cloud_proj_z);
      
	  
      /* ____________________________
	|                           |
	|  Axes sign disambiguation |
	|___________________________| */
			
      
	compute_bounding_box_dimensions(target_pc, dimensions);
	ROS_INFO("Box's dimensions (x, y, z) = (%f, %f, %f) ", dimensions.x, dimensions.y, dimensions.z);
	//double largest_side;
	compute_largest_side_of_bounding_box(dimensions ,largest_side);		
	ROS_INFO("dimension of largest_side =  %f ", largest_side);

	if (adaptive_support_lenght == 0)
	{
	  ROS_INFO("Dimension of the (largest_side+0.02)/2 is %f m", largest_side/2);
	  ROS_INFO("adaptive_support_lenght = %d", adaptive_support_lenght);
	}
	else
	{
	  largest_side = global_image_width;
	  ROS_INFO("global_image_width = %f", largest_side);
	  ROS_INFO("adaptive_support_lenght = %d", adaptive_support_lenght);
	}
	  
      /* ____________________________
	|                           |
	|  Axes sign disambiguation |
	|___________________________| */
    
	int Sx=1, Sy=1;
	XsignDisambiguation(initial_cloud_proj_y, threshold, Sx );//XoZ Plane
	YsignDisambiguation(initial_cloud_proj_x, threshold, Sy );//YoZ Plane
	sign = Sx * Sy;
	ROS_INFO("sign = Sx (%d) * Sy(%d) = %d", Sx, Sy, sign);
	ROS_INFO("signDisambiguationFlag is updated");

	//eigVectors.col(2) = sign * eigVectors.col(2);
	
	// eigVectors.col(0)= Sx * eigVectors.col(0);
// 	 eigVectors.col(1)= Sy * eigVectors.col(1);
// 	 eigVectors.col(2) = eigVectors.col(0).cross(eigVectors.col(1));
// 	 
// 	 // move the points to the PCA based reference frame
// 	  //p2w(Eigen::Matrix4f::Identity());
// 	  p2w.block<3,3>(0,0) = eigVectors.transpose();
// 	  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
// 	  
// 	  pcl::transformPointCloud(*target_pc, *pca_object_view, p2w);
// 	  //compute the max, the min and the center of the diagonal (mean_diag)
// 	  pcl::getMinMax3D(*pca_object_view, min_pt, max_pt);
// 	  const Eigen::Vector3f mean_diag2 = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
// 	  // centroid transform
// // // 	  Eigen::Quaternionf qfinal(eigVectors);//rotation matrix
// // 	  center_of_bbox = eigVectors*mean_diag2 + centroid.head<3>(); // Translation = Rotation * center_diag + (c0, c1, c2)
// // 	  std::cout << "center of box (x,y,z) =" << center_of_bbox << std::endl;
// // // 


    /* _________________________________________________________
      |                       				       	  |
      |  compute histograms of projection of the given object   |
      |_________________________________________________________| */
	
	vector <int> complete_object_histogram;
	vector <int> complete_object_histogram_normalized;//each projection view is normalized sepreatly
	vector < vector<int> > XOZ_histogram;      
	vector < vector<int> > YOZ_histogram;
// 	vector <float> view_point_entropy;
	vector <vector <float> > normalized_projected_views;

	//projection along X axis
	YOZ2DObjectHistogram( initial_cloud_proj_x,
			      largest_side, 
			      number_of_bins, 
			      sign,
			      YOZ_histogram);
	
	vector <int> histogramYOZ1D;
	convert2DhistogramTo1Dhisftogram(YOZ_histogram, histogramYOZ1D);
	//printHistogram ( histogramYOZ1D, "YOZ");
	complete_object_histogram.insert(complete_object_histogram.end(), histogramYOZ1D.begin(), histogramYOZ1D.end());
	vector <float> normalized_histogramYoZ;
	normalizingHistogram( histogramYOZ1D, normalized_histogramYoZ);
	normalized_projected_views.push_back(normalized_histogramYoZ);
	
	//printHistogram ( normalized_histogramYoZ, "normalized YOZ");
	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), normalized_histogramYoZ.begin(), normalized_histogramYoZ.end());
	float YoZ_entropy = 0;
	//viewpointEntropy(normalized_histogramYoZ, YoZ_entropy);
	viewpointEntropyNotNormalized(histogramYOZ1D, YoZ_entropy);

	ROS_INFO("viewpointEntropyYoZ = %f", YoZ_entropy);
	view_point_entropy.push_back(YoZ_entropy);

	//projection along Y axis
	XOZ2DObjectHistogram( initial_cloud_proj_y,
			      largest_side, 
			      number_of_bins, 
			      sign,
			      XOZ_histogram);
      
	vector <int> histogramXOZ1D;
	convert2DhistogramTo1Dhisftogram(XOZ_histogram, histogramXOZ1D);
	//printHistogram ( histogramXOZ1D, "XOZ");
	complete_object_histogram.insert(complete_object_histogram.end(), histogramXOZ1D.begin(), histogramXOZ1D.end());

	vector <float> normalized_histogramXoZ;
	normalizingHistogram( histogramXOZ1D, normalized_histogramXoZ);
	//printHistogram ( normalized_histogramXoZ, "normalized XOZ");
	normalized_projected_views.push_back(normalized_histogramXoZ);

	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), 
						  normalized_histogramXoZ.begin(), 
						  normalized_histogramXoZ.end());
	float XoZ_entropy = 0;
	//viewpointEntropy(normalized_histogramXoZ, XoZ_entropy);
	viewpointEntropyNotNormalized(histogramXOZ1D, XoZ_entropy);
	ROS_INFO("viewpointEntropyXoZ = %f", XoZ_entropy);
	view_point_entropy.push_back(XoZ_entropy);


	//projection along Z axis
	vector < vector<int> > XOY_histogram;		
	XOY2DObjectHistogram( initial_cloud_proj_z,
				largest_side, 
				number_of_bins, 
				sign,
				XOY_histogram);

	vector <int> histogramXOY1D;
	convert2DhistogramTo1Dhisftogram(XOY_histogram, histogramXOY1D);
	//printHistogram ( histogramXOY1D, "XOY");		
	complete_object_histogram.insert(complete_object_histogram.end(), histogramXOY1D.begin(), histogramXOY1D.end());
	//printHistogram ( complete_object_histogram, "complete_object_histogram");
	
	vector <float> normalized_histogramXoY;
	normalizingHistogram( histogramXOY1D, normalized_histogramXoY);
	//printHistogram ( normalized_histogramXoY, "normalized XoY");
	normalized_projected_views.push_back(normalized_histogramXoY);

	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), normalized_histogramXoY.begin(), normalized_histogramXoY.end());
	float XoY_entropy = 0;
	//viewpointEntropy(normalized_histogramXoY, XoY_entropy);
	viewpointEntropyNotNormalized(histogramXOY1D, XoY_entropy);

	ROS_INFO("viewpointEntropyXoY = %f", XoY_entropy);
	view_point_entropy.push_back(XoY_entropy);

	//printHistogram ( complete_object_histogram_normalized, "complete_object_histogram_normalized");
	
	vector <float> normalized_histogram;
	normalizingHistogram( complete_object_histogram, normalized_histogram);
	//printHistogram ( normalized_histogram, "normalized_complete_object_histogram");

	int maximum_entropy_index = 0;
	findMaxViewPointsEntropy(view_point_entropy, maximum_entropy_index);
	ROS_INFO("Summary of entropy : \n H(YoZ) = %f, H(XoZ) = %f, H(XoY) = %f, Max_ind = %d", 
		  view_point_entropy.at(0), view_point_entropy.at(1), view_point_entropy.at(2) , maximum_entropy_index );		
	
// 	vector< float > object_description;
// 	string std_name_of_sorted_projected_plane;
	objectViewHistogram( maximum_entropy_index,
			    view_point_entropy,
			    normalized_projected_views,
			    object_description,
			    std_name_of_sorted_projected_plane
			  );
	
	ROS_INFO("Projected views are sorted as follows: %s", std_name_of_sorted_projected_plane.c_str());   		
	//printHistogram ( sorted_normalized_projected_views, "sorted_normalized_projected_views");
	

	return 0; 
}





int compuet_object_description_2D_variance( boost::shared_ptr<pcl::PointCloud<T> > target_pc,
					      int adaptive_support_lenght,
					      double global_image_width,
					      int threshold,
					      int number_of_bins,
					      boost::shared_ptr<pcl::PointCloud<T> > &pca_object_view,
					      Eigen::Vector3f &center_of_bbox,
					      vector < boost::shared_ptr<pcl::PointCloud<T> > > &vector_of_projected_views, 
					      double &largest_side, 
					      int &sign,
					      vector <float> &view_point_entropy,
					      string &std_name_of_sorted_projected_plane,
					      vector< float > &object_description,
					      int sorting_criterion
					  )	
{


	ROS_INFO("given point cloud has %d points ", target_pc->points.size());
      
// 	float downsampling_voxel_size= 0.001;
// 	downSampling ( target_pc,  downsampling_voxel_size,  target_pc);
// 	
	ROS_INFO("//////////////////////////////// compuet_object_description ///////////////////////////////////////////");

	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_x (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_y (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_z (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	bool visualize = false;
	PointXYZ pt; 
	
	/* ____________________________
	  |                            |
	  | construct ORF based on PCA |
	  |____________________________| */
	
	  ///////////////////////////////////////////////// the theory of new shape descriptor ////////////////////////////////////////////////////////////////
	  // //NOTE  the PCA base reference frame construction basically does:
	  // 1) compute the centroid (c0, c1, c2) and the normalized covariance
	  // 2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
	  // 3) move the points in that RF --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
	  // 4) compute the max, the min and the center of the diagonal (mean_diag)
	  // 5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) 
	  //    the transformation you have to apply is Rotation = (e0, e1, e0 X e1) & Translation = Rotation * mean_diag + (c0, c1, c2)

	  
	  // compute principal directions
	  Eigen::Vector4f centroid;
	  pcl::compute3DCentroid(*target_pc, centroid);
	  Eigen::Matrix3f covariance;
	  computeCovarianceMatrixNormalized(*target_pc, centroid, covariance);
	  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	  Eigen::Matrix3f eigVectors = eigen_solver.eigenvectors();
	  eigVectors.col(2) = eigVectors.col(0).cross(eigVectors.col(1));
	  std::cout << "eigen vectores :  \n " << eigVectors << std::endl;


// 	  std::cout << "eigen vectores \n " << eigVectors << std::endl;
// 	  std::cout << "column1: " << eigVectors.col(0) << std::endl;
// 	  std::cout << "column2: " << eigVectors.col(1) << std::endl;
// 	  std::cout << "column3: " << eigVectors.col(2) << std::endl;
	  
	  setprecision(20);
	  Eigen::Vector3f eigen_values =  eigen_solver.eigenvalues();
	  std::cout << "The eigenvalues of the covariance matrix before sorting are \n:" << eigen_values << endl;
// 	  std::cout << "The eigenvalue of the z axis is :" << eigen_values(0,0) << endl;
// 	  std::cout << "The eigenvalue of the Y axis is :" << eigen_values(0,1) << endl;
// 	  std::cout << "The eigenvalue of the x axis is :" << eigen_values(0,2) << endl;
// 	  ROS_INFO("eigenvalue(0) = %f \t eigenvalue(1) = %f \t eigenvalue(2) = %f", eigen_values(0,0),  eigen_values(0,1),  eigen_values(0,2) );
		  	  
// 	  std::sort(eigenValues.begin(), eigenValues.end());
//	  std::cout <<std::setprecision(4)<<"eigen vectores after sorting: " << eigenValues << endl;
	  
	  Eigen::Matrix3f eigVectors_tmp;
	  Eigen::Vector3f eigen_values_tmp;
// 	  //sorting eigen vectors based on eigen values
	  eigVectors.col(0)= eigVectors.col(2);
	  eigVectors.col(2) = eigVectors.col(0).cross(eigVectors.col(1));

	  std::cout << "eigen vectores cross product :  \n " << eigVectors << std::endl;

	  // 	  std::cout << "eigen vectores after sorting \n " << eigVectors << std::endl;
// 	  std::cout << "column1 after sort: " << eigVectors.col(0) << std::endl;
// 	  std::cout << "column2 after sort: " << eigVectors.col(1) << std::endl;
// 	  std::cout << "column3 after sort: " << eigVectors.col(2) << std::endl;

// 	  std::cout << "The eigenvalues of the covariance matrix after sorting are: \n" << eigen_values << endl;
// 	  std::cout << "The eigenvalue of the X axis is :" << eigen_values(0,0) << endl;
// 	  std::cout << "The eigenvalue of the Y axis is :" << eigen_values(0,1) << endl;
// 	  std::cout << "The eigenvalue of the Z axis is :" << eigen_values(0,2) << endl;
	  
	  

	  // move the points to the PCA based reference frame
	  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
	  p2w.block<3,3>(0,0) = eigVectors.transpose();
	  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
	  
	  //Declare a boost share ptr to the PCA_pointCloud
// 	  boost::shared_ptr<PointCloud<PointT> > pca_pc (new PointCloud<PointT>); 
		 
  
	  pcl::PointCloud<PointT> cPoints;
	  pcl::transformPointCloud(*target_pc, *pca_object_view, p2w);
	  //compute the max, the min and the center of the diagonal (mean_diag)
	  PointT min_pt, max_pt;
	  pcl::getMinMax3D(*pca_object_view, min_pt, max_pt);
	  const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
	  // centroid transform
	  Eigen::Quaternionf qfinal(eigVectors);//rotation matrix
	  center_of_bbox = eigVectors*mean_diag + centroid.head<3>(); // Translation = Rotation * center_diag + (c0, c1, c2)
	  std::cout << "center of box (x,y,z) =" << center_of_bbox << std::endl;

      /* _________________________________
	|                           	   |
	| construct three projection view |
	|_________________________________| */
			
	// ax+by+cz+d=0, where b=c=d=0, and a=1, or said differently, the YoZ plane.
	pcl::ModelCoefficients::Ptr coefficientsX (new pcl::ModelCoefficients ());
	coefficientsX->values.resize (4);
	coefficientsX->values[0] = 1.0; coefficientsX->values[1] = 0; coefficientsX->values[2] = 0; coefficientsX->values[3] = 0;
	project_pc_to_plane(pca_object_view, coefficientsX, initial_cloud_proj_x);
	for (int i=0; i< initial_cloud_proj_x->points.size(); i++)
	{
	    initial_cloud_proj_x->points.at(i).x = 0.3;
	}
	vector_of_projected_views.push_back(initial_cloud_proj_x);

	
	//ax+by+cz+d=0, where a=c=d=0, and b=1, or said differently, the XoZ plane.
	pcl::ModelCoefficients::Ptr coefficientsY (new pcl::ModelCoefficients ());
	coefficientsY->values.resize (4);
	coefficientsY->values[0] = 0.0; coefficientsY->values[1] = 1.0; coefficientsY->values[2] = 0; coefficientsY->values[3] = 0;

	project_pc_to_plane(pca_object_view, coefficientsY, initial_cloud_proj_y);
	for (int i=0; i< initial_cloud_proj_y->points.size(); i++)
	{
	    initial_cloud_proj_y->points.at(i).y = 0.3;
	}
	vector_of_projected_views.push_back(initial_cloud_proj_y);

		  
	// ax+by+cz+d=0, where a=b=d=0, and c=1, or said differently, the XoY plane.
	pcl::ModelCoefficients::Ptr coefficientsZ (new pcl::ModelCoefficients ());
	coefficientsZ->values.resize (4); 
	coefficientsZ->values[0] = 0; coefficientsZ->values[1] = 0; coefficientsZ->values[2] = 1.0;   coefficientsZ->values[3] = 0;
	project_pc_to_plane(pca_object_view, coefficientsZ, initial_cloud_proj_z);
	for (int i=0; i< initial_cloud_proj_z->points.size(); i++)
	{
	    initial_cloud_proj_z->points.at(i).z = 0.30;
	}		
	vector_of_projected_views.push_back(initial_cloud_proj_z);
      
	  
      /* ____________________________
	|                           |
	|  Axes sign disambiguation |
	|___________________________| */
			
	geometry_msgs::Vector3 dimensions;
	compute_bounding_box_dimensions(pca_object_view, dimensions);
	ROS_INFO("Box's dimensions (x, y, z) = (%f, %f, %f) ", dimensions.x, dimensions.y, dimensions.z);
	//double largest_side;
	compute_largest_side_of_bounding_box(dimensions ,largest_side);		

	if (adaptive_support_lenght == 0)
	{
	  ROS_INFO("Dimension of the (largest_side+0.02)/2 is %f m", largest_side/2);
	  ROS_INFO("adaptive_support_lenght = %d", adaptive_support_lenght);
	}
	else
	{
	  largest_side = global_image_width;
	  ROS_INFO("global_image_width = %f", largest_side);
	  ROS_INFO("adaptive_support_lenght = %d", adaptive_support_lenght);
	}
	  
      /* ____________________________
	|                           |
	|  Axes sign disambiguation |
	|___________________________| */
    
	int Sx=1, Sy=1;
	XsignDisambiguation(initial_cloud_proj_y, threshold, Sx );//XoZ Plane
	YsignDisambiguation(initial_cloud_proj_x, threshold, Sy );//YoZ Plane
	sign = Sx * Sy;
	ROS_INFO("sign = Sx (%d) * Sy(%d) = %d", Sx, Sy, sign);
	ROS_INFO("signDisambiguationFlag is updated");

	//eigVectors.col(2) = sign * eigVectors.col(2);
	
	// eigVectors.col(0)= Sx * eigVectors.col(0);
// 	 eigVectors.col(1)= Sy * eigVectors.col(1);
// 	 eigVectors.col(2) = eigVectors.col(0).cross(eigVectors.col(1));
// 	 
// 	 // move the points to the PCA based reference frame
// 	  //p2w(Eigen::Matrix4f::Identity());
// 	  p2w.block<3,3>(0,0) = eigVectors.transpose();
// 	  p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
// 	  
// 	  pcl::transformPointCloud(*target_pc, *pca_object_view, p2w);
// 	  //compute the max, the min and the center of the diagonal (mean_diag)
// 	  pcl::getMinMax3D(*pca_object_view, min_pt, max_pt);
// 	  const Eigen::Vector3f mean_diag2 = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());
// 	  // centroid transform
// // // 	  Eigen::Quaternionf qfinal(eigVectors);//rotation matrix
// // 	  center_of_bbox = eigVectors*mean_diag2 + centroid.head<3>(); // Translation = Rotation * center_diag + (c0, c1, c2)
// // 	  std::cout << "center of box (x,y,z) =" << center_of_bbox << std::endl;
// // // 


    /* _________________________________________________________
      |                       				       	  |
      |  compute histograms of projection of the given object   |
      |_________________________________________________________| */
	
	vector <int> complete_object_histogram;
	vector <int> complete_object_histogram_normalized;//each projection view is normalized sepreatly
	vector < vector<int> > XOZ_histogram;      
	vector < vector<int> > YOZ_histogram;
// 	vector <float> view_point_entropy;
	vector <vector <float> > normalized_projected_views;

	vector < vector < vector<float> > > normalized_2D_projected_views;

	//projection along X axis
	YOZ2DObjectHistogram( initial_cloud_proj_x,
			      largest_side, 
			      number_of_bins, 
			      sign,
			      YOZ_histogram);

	
	vector < vector<float> > normalized_YOZ_histogram;
	normalized2Dhistogram (YOZ_histogram, normalized_YOZ_histogram,target_pc->points.size());
	normalized_2D_projected_views.push_back(normalized_YOZ_histogram);
	
	
	vector <int> histogramYOZ1D;
	convert2DhistogramTo1Dhisftogram(YOZ_histogram, histogramYOZ1D);
	//printHistogram ( histogramYOZ1D, "YOZ");
	complete_object_histogram.insert(complete_object_histogram.end(), histogramYOZ1D.begin(), histogramYOZ1D.end());
	vector <float> normalized_histogramYoZ;
	normalizingHistogram( histogramYOZ1D, normalized_histogramYoZ);
	normalized_projected_views.push_back(normalized_histogramYoZ);
	
	//printHistogram ( normalized_histogramYoZ, "normalized YOZ");
	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), normalized_histogramYoZ.begin(), normalized_histogramYoZ.end());
	float YoZ_entropy = 0;
	//viewpointEntropy(normalized_histogramYoZ, YoZ_entropy);
	viewpointEntropyNotNormalized(histogramYOZ1D, YoZ_entropy);

	ROS_INFO("viewpointEntropyYoZ = %f", YoZ_entropy);
	view_point_entropy.push_back(YoZ_entropy);
	
	//projection along Y axis
	XOZ2DObjectHistogram( initial_cloud_proj_y,
			      largest_side, 
			      number_of_bins, 
			      sign,
			      XOZ_histogram);
	
	vector < vector<float> > normalized_XOZ_histogram;
	normalized2Dhistogram (XOZ_histogram, normalized_XOZ_histogram,target_pc->points.size());
	normalized_2D_projected_views.push_back(normalized_XOZ_histogram);
	
	
	vector <int> histogramXOZ1D;
	convert2DhistogramTo1Dhisftogram(XOZ_histogram, histogramXOZ1D);
	//printHistogram ( histogramXOZ1D, "XOZ");
	complete_object_histogram.insert(complete_object_histogram.end(), histogramXOZ1D.begin(), histogramXOZ1D.end());

	vector <float> normalized_histogramXoZ;
	normalizingHistogram( histogramXOZ1D, normalized_histogramXoZ);
	//printHistogram ( normalized_histogramXoZ, "normalized XOZ");
	normalized_projected_views.push_back(normalized_histogramXoZ);

	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), 
						  normalized_histogramXoZ.begin(), 
						  normalized_histogramXoZ.end());
	float XoZ_entropy = 0;
	//viewpointEntropy(normalized_histogramXoZ, XoZ_entropy);
	viewpointEntropyNotNormalized(histogramXOZ1D, XoZ_entropy);
	ROS_INFO("viewpointEntropyXoZ = %f", XoZ_entropy);
	view_point_entropy.push_back(XoZ_entropy);


	//projection along Z axis
	vector < vector<int> > XOY_histogram;		
	XOY2DObjectHistogram( initial_cloud_proj_z,
				largest_side, 
				number_of_bins, 
				sign,
				XOY_histogram);

	vector < vector<float> > normalized_XOY_histogram;
	normalized2Dhistogram (XOY_histogram, normalized_XOY_histogram,target_pc->points.size());
	normalized_2D_projected_views.push_back(normalized_XOY_histogram);
	
	
	vector <int> histogramXOY1D;
	convert2DhistogramTo1Dhisftogram(XOY_histogram, histogramXOY1D);
	//printHistogram ( histogramXOY1D, "XOY");		
	complete_object_histogram.insert(complete_object_histogram.end(), histogramXOY1D.begin(), histogramXOY1D.end());
	//printHistogram ( complete_object_histogram, "complete_object_histogram");
	
	vector <float> normalized_histogramXoY;
	normalizingHistogram( histogramXOY1D, normalized_histogramXoY);
	//printHistogram ( normalized_histogramXoY, "normalized XoY");
	normalized_projected_views.push_back(normalized_histogramXoY);

	complete_object_histogram_normalized.insert(complete_object_histogram_normalized.end(), normalized_histogramXoY.begin(), normalized_histogramXoY.end());
	float XoY_entropy = 0;
	//viewpointEntropy(normalized_histogramXoY, XoY_entropy);
	viewpointEntropyNotNormalized(histogramXOY1D, XoY_entropy);

	ROS_INFO("viewpointEntropyXoY = %f", XoY_entropy);
	view_point_entropy.push_back(XoY_entropy);

	//printHistogram ( complete_object_histogram_normalized, "complete_object_histogram_normalized");
	
	vector <float> normalized_histogram;
	normalizingHistogram( complete_object_histogram, normalized_histogram);
	//printHistogram ( normalized_histogram, "normalized_complete_object_histogram");
	bool weighted_variance = false;
	int maximum_entropy_index = 0;
	
	


	switch(sorting_criterion) {
	  
	  case SORT_BY_ENTROPY_AND_1D_VARIANCE:
	    break;

	  case SORT_BY_ENTROPY_AND_AVERAGE_DISTANCE:
	    break;
	    
	  case SORT_BY_ENTROPY_AND_2D_STDDEV_SUM:
	
	    



	    
		maximum_entropy_index = 0;
		findMaxViewPointsEntropy(view_point_entropy, maximum_entropy_index);
		ROS_INFO("Summary of entropy : \n H(YoZ) = %f, H(XoZ) = %f, H(XoY) = %f, Max_ind = %d", 
			  view_point_entropy.at(0), view_point_entropy.at(1), view_point_entropy.at(2) , maximum_entropy_index );		
		
	// 	vector< float > object_description;
	// 	string std_name_of_sorted_projected_plane;
	// 	vector< float > object_description1D;
	// 	objectViewHistogram( maximum_entropy_index,
	// 			    view_point_entropy,
	// 			    normalized_projected_views,
	// 			    object_description1D,
	// 			    std_name_of_sorted_projected_plane
	// 			  );
	// 	ROS_INFO("Projected views are sorted using 1D variance as follows: %s", std_name_of_sorted_projected_plane.c_str());   		
		ROS_INFO("BEFORE objectViewHistogramUsing2Dvariance ");
		objectViewHistogramUsing2Dvariance(  maximum_entropy_index,
						      normalized_projected_views.at(maximum_entropy_index),
						      normalized_projected_views /*all projection_views[0,1,2]*/,
						      normalized_2D_projected_views,
						      object_description,
						      weighted_variance,
						      std_name_of_sorted_projected_plane /*debug*/
						    );
		break;

	  	  case SORT_BY_ENTROPY_AND_2D_VARIANCE_SUM:
	
			maximum_entropy_index = 0;
			findMaxViewPointsEntropy(view_point_entropy, maximum_entropy_index);
			ROS_INFO("Summary of entropy : \n H(YoZ) = %f, H(XoZ) = %f, H(XoY) = %f, Max_ind = %d", 
				  view_point_entropy.at(0), view_point_entropy.at(1), view_point_entropy.at(2) , maximum_entropy_index );		
			
		// 	vector< float > object_description;
		// 	string std_name_of_sorted_projected_plane;
		// 	vector< float > object_description1D;
		// 	objectViewHistogram( maximum_entropy_index,
		// 			    view_point_entropy,
		// 			    normalized_projected_views,
		// 			    object_description1D,
		// 			    std_name_of_sorted_projected_plane
		// 			  );
		// 	ROS_INFO("Projected views are sorted using 1D variance as follows: %s", std_name_of_sorted_projected_plane.c_str());   		
			weighted_variance = true;
			ROS_INFO("BEFORE objectViewHistogramUsing2Dvariance ");
			objectViewHistogramUsing2Dvariance(  maximum_entropy_index,
							      normalized_projected_views.at(maximum_entropy_index),
							      normalized_projected_views /*all projection_views[0,1,2]*/,
							      normalized_2D_projected_views,
							      object_description,
							      weighted_variance,
							      std_name_of_sorted_projected_plane /*debug*/
							    );
		break;

	  
	  case SORT_BY_DISTINCTIVENESS:
	    objectViewHistogramUsingDistinctiveness( normalized_2D_projected_views,
						      object_description,
						      std_name_of_sorted_projected_plane );
	    break;
	}

	
	ROS_INFO("Projected views are sorted using 2D variance as follows: %s", std_name_of_sorted_projected_plane.c_str());   		
	//printHistogram ( sorted_normalized_projected_views, "sorted_normalized_projected_views");
	

	return 0; 
}


int compuet_object_description_for_grasp( boost::shared_ptr<pcl::PointCloud<T> > pca_object_view,
					  int number_of_bins,
					  vector< float > &object_description )	
{
	
	int sign = 1;
	double largest_side; 
	vector < boost::shared_ptr<pcl::PointCloud<T> > > vector_of_projected_views; 
	//vector < boost::shared_ptr<pcl::PointCloud<T> > > &vector_of_projected_views, 
	ROS_INFO("//////////////////////////////// compuet_object_description_for_grasp ///////////////////////////////////////////");

	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_x (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_y (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	boost::shared_ptr<pcl::PointCloud<PointT> > initial_cloud_proj_z (new PointCloud<PointT>);//Declare a boost share ptr to the pointCloud
	bool visualize = false;
	PointXYZ pt; 

      /* _________________________________
	|                           	  |
	| construct three projection view |
	|_________________________________| */
			
	// ax+by+cz+d=0, where b=c=d=0, and a=1, or said differently, the YoZ plane.
	pcl::ModelCoefficients::Ptr coefficientsX (new pcl::ModelCoefficients ());
	coefficientsX->values.resize (4);
	coefficientsX->values[0] = 1.0; coefficientsX->values[1] = 0; coefficientsX->values[2] = 0; coefficientsX->values[3] = 0;
	project_pc_to_plane(pca_object_view, coefficientsX, initial_cloud_proj_x);
	for (int i=0; i< initial_cloud_proj_x->points.size(); i++)
	{
	    initial_cloud_proj_x->points.at(i).x = 0.3;
	}
	vector_of_projected_views.push_back(initial_cloud_proj_x);

	
	//ax+by+cz+d=0, where a=c=d=0, and b=1, or said differently, the XoZ plane.
	pcl::ModelCoefficients::Ptr coefficientsY (new pcl::ModelCoefficients ());
	coefficientsY->values.resize (4);
	coefficientsY->values[0] = 0.0; coefficientsY->values[1] = 1.0; coefficientsY->values[2] = 0; coefficientsY->values[3] = 0;

	project_pc_to_plane(pca_object_view, coefficientsY, initial_cloud_proj_y);
	for (int i=0; i< initial_cloud_proj_y->points.size(); i++)
	{
	    initial_cloud_proj_y->points.at(i).y = 0.3;
	}
	vector_of_projected_views.push_back(initial_cloud_proj_y);
		  
	// ax+by+cz+d=0, where a=b=d=0, and c=1, or said differently, the XoY plane.
	pcl::ModelCoefficients::Ptr coefficientsZ (new pcl::ModelCoefficients ());
	coefficientsZ->values.resize (4); 
	coefficientsZ->values[0] = 0; coefficientsZ->values[1] = 0; coefficientsZ->values[2] = 1.0;   coefficientsZ->values[3] = 0;
	project_pc_to_plane(pca_object_view, coefficientsZ, initial_cloud_proj_z);
	for (int i=0; i< initial_cloud_proj_z->points.size(); i++)
	{
	    initial_cloud_proj_z->points.at(i).z = 0.30;
	}		
	vector_of_projected_views.push_back(initial_cloud_proj_z);
      
	  
      /* _______________________________________
	|                         		|
	|  compute_largest_side_of_bounding_box |
	|_______________________________________| */
			
	geometry_msgs::Vector3 dimensions;
	compute_bounding_box_dimensions(pca_object_view, dimensions);
	ROS_INFO("Box's dimensions (x, y, z) = (%f, %f, %f) ", dimensions.x, dimensions.y, dimensions.z);
	//double largest_side;
	compute_largest_side_of_bounding_box(dimensions ,largest_side);		
  
    /* _________________________________________________________
      |                       				  	|
      |  compute histograms of projection of the given object   |
      |_________________________________________________________| */
	
	vector <int> complete_object_histogram;
	vector <int> complete_object_histogram_normalized;//each projection view is normalized sepreatly
	vector < vector<int> > XOZ_histogram;      
	vector < vector<int> > YOZ_histogram;
	vector <float> normalized_histogramXoZ;
	
	//projection along X axis
	YOZ2DObjectHistogram( initial_cloud_proj_x,
			      largest_side, 
			      number_of_bins, 
			      sign,
			      YOZ_histogram);
	
	vector <int> histogramYOZ1D;
	convert2DhistogramTo1Dhisftogram(YOZ_histogram, histogramYOZ1D);
	complete_object_histogram.insert(complete_object_histogram.end(), histogramYOZ1D.begin(), histogramYOZ1D.end());
	vector <float> normalized_histogramYoZ;
	normalizingHistogram( histogramYOZ1D, normalized_histogramYoZ);
	object_description.insert(object_description.end(), normalized_histogramYoZ.begin(), normalized_histogramYoZ.end());

	//projection along Y axis
	XOZ2DObjectHistogram( initial_cloud_proj_y,
			      largest_side, 
			      number_of_bins, 
			      sign,
			      XOZ_histogram);
      
	vector <int> histogramXOZ1D;
	convert2DhistogramTo1Dhisftogram(XOZ_histogram, histogramXOZ1D);
	complete_object_histogram.insert(complete_object_histogram.end(), histogramXOZ1D.begin(), histogramXOZ1D.end());
	normalizingHistogram( histogramXOZ1D, normalized_histogramXoZ);
	object_description.insert(object_description.end(), normalized_histogramXoZ.begin(), normalized_histogramXoZ.end());

	//projection along Z axis
	vector < vector<int> > XOY_histogram;		
	XOY2DObjectHistogram( initial_cloud_proj_z,
				largest_side, 
				number_of_bins, 
				sign,
				XOY_histogram);

	vector <int> histogramXOY1D;
	convert2DhistogramTo1Dhisftogram(XOY_histogram, histogramXOY1D);
	complete_object_histogram.insert(complete_object_histogram.end(), histogramXOY1D.begin(), histogramXOY1D.end());
	vector <float> normalized_histogramXoY;
	normalizingHistogram( histogramXOY1D, normalized_histogramXoY);
	object_description.insert(object_description.end(), normalized_histogramXoY.begin(), normalized_histogramXoY.end());
	
	return 0; 
}




#endif
