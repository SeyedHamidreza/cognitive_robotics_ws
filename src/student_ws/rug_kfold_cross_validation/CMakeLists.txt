cmake_minimum_required(VERSION 3.0.2)
project(rug_kfold_cross_validation)

#Find PCL 1.8
find_package(PCL 1.8 REQUIRED)
if (NOT PCL_FOUND)
	MESSAGE("PCL 1.8 not found.")
endif (NOT PCL_FOUND)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Boost COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
find_package(OpenCV REQUIRED )
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


set(PKG_DEPS
	roscpp
	std_msgs
	roslib
	nodelet
	sensor_msgs
	race_perception_msgs
	race_perception_db
	race_feature_extraction
	race_object_descriptor
	race_object_conceptualizer
	race_perception_utils
	pcl_conversions
	race_perception_db
 	race_perception_utils
 	rug_deep_feature_extraction
	cv_bridge
)


find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS} 	
	${PCL_INCLUDE_DIRS}
)




##################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
	INCLUDE_DIRS include 
	LIBRARIES rug_kfold_cross_validation_functionality 
	CATKIN_DEPENDS ${PKG_DEPS}
)

###Library
add_library(rug_kfold_cross_validation_functionality src/rug_kfold_cross_validation_functionality.cpp)
target_link_libraries(rug_kfold_cross_validation_functionality ${catkin_LIBRARIES})
target_link_libraries(rug_kfold_cross_validation_functionality ${PCL_LIBRARIES})
target_link_libraries(rug_kfold_cross_validation_functionality ${OpenCV_LIBS}) 
target_link_libraries(rug_kfold_cross_validation_functionality boost_system)


###Executable

## hand-crafted
add_executable(k_fold_cross_validation_hand_crafted_descriptors src/k_fold_cross_validation_hand_crafted_descriptors.cpp)
target_link_libraries(k_fold_cross_validation_hand_crafted_descriptors ${catkin_LIBRARIES})
#target_link_libraries(k_fold_cross_validation_hand_crafted_descriptors ${PCL_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(k_fold_cross_validation_hand_crafted_descriptors rug_kfold_cross_validation_functionality)
target_link_libraries(k_fold_cross_validation_hand_crafted_descriptors boost_system)


## deep transfer learning
add_executable(k_fold_cross_validation_RGBD_deep_learning_descriptor src/k_fold_cross_validation_RGBD_deep_learning_descriptor.cpp)
target_link_libraries(k_fold_cross_validation_RGBD_deep_learning_descriptor ${catkin_LIBRARIES})
#target_link_libraries(k_fold_cross_validation_RGBD_deep_learning_descriptor ${PCL_LIBRARIES} ${PCL_INCLUDE_DIRS})
target_link_libraries(k_fold_cross_validation_RGBD_deep_learning_descriptor rug_kfold_cross_validation_functionality)
target_link_libraries(k_fold_cross_validation_RGBD_deep_learning_descriptor boost_system)




