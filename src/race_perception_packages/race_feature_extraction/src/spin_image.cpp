#ifndef _SPIN_IMAGE_CPP_
#define _SPIN_IMAGE_CPP_

//includes
#include <feature_extraction/spin_image.h>
#include <CGAL/Plane_3.h>
#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/features/vfh.h>

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

//     ROS_INFO("TEST3####: uniform_keypoints.size = %d", uniform_keypoints->size());

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
		
	      if(std::isnan(spin_images->points.at(uniform_sampling_indices->at(i) ).histogram[ j ] ))
	      {
		  tmp.spin_image.push_back(0);
		  ROS_INFO ("There is a NaN number in the spin images");
	      }
	      else
	      {
		  tmp.spin_image.push_back( spin_images->points.at( uniform_sampling_indices->at(i) ).histogram[ j ] );
	      }
	     
	    }
	    spin_images_msg->push_back( tmp );	
	}
	      	
// 		for (size_t i = 0; i < spin_images->size(); i++) // remove NAN values from the computed spin image (replace them by 0)  
// 	    {
// 		for (size_t j = 0; j < spin_image_size; j++)
// 		{
// 			if(std::isnan(spin_images->points.at(i).histogram[j]))
// 			{
// 				spin_images->points.at(i).histogram[j]=0;
// 			}
// 		}
// 	}
	
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




int KLDifferenceBetweenSpinImage( SITOV  sp1, 
				   SITOV  sp2,
				   float &difference)	
{
	//ROS_INFO("\t\t[-]- Inside differenceBetweenSpinImage function");
	difference = 0;
	float Distance_Q_P = 0;
	float Distance_P_Q = 0;
	//ROS_INFO("\t\t[-]- sp1 size = %ld , sp2 size = %ld", sp1.spin_image.size(), sp2.spin_image.size());
	if (sp1.spin_image.size() != sp2.spin_image.size())
	{
		return 0;
	}

	for (size_t i = 0; i < sp1.spin_image.size(); i++)
	{
		
	  if (sp2.spin_image.at(i) != 0)
	  {
	      Distance_Q_P += sp1.spin_image.at(i)* (log2(sp1.spin_image.at(i)/sp2.spin_image.at(i)));
	      
	  }
	  
	  if (sp1.spin_image.at(i) != 0)
	  {
	      Distance_P_Q += sp2.spin_image.at(i)* (log2(sp2.spin_image.at(i)/sp1.spin_image.at(i)));
	  }
	 
	}
	 difference = 0.5 * ( Distance_P_Q + Distance_Q_P);

	return 1;
}

int KLdifferenceBetweenObjectViews( std::vector< SITOV> spin_images1, 
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
		
		if ( !KLDifferenceBetweenSpinImage( sp1, sp2, si_difference) )		    
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
}//functions


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

#endif


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
	//uniform_sampling.compute (sampled_indices);//PCL 1.7;
	//copyPointCloud (*cloud, sampled_indices.points, *downsampled_pc);//Keypoints = voxel grid downsampling //PCL 1.7;

	uniform_sampling.filter (*downsampled_pc);//indices means "fehrest";




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


