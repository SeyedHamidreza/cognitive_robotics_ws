cmake_minimum_required(VERSION 3.0.2)
project(rug_simulated_user)

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
	rospy
	std_msgs
	roslib
	nodelet
	geometry_msgs
	sensor_msgs
	race_perception_msgs
	race_perception_db
	race_feature_extraction
	race_object_descriptor
 	race_object_conceptualizer
 	race_perception_utils
	pcl_conversions
	rug_kfold_cross_validation
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
	LIBRARIES rug_simulated_user_functionality 
	CATKIN_DEPENDS ${PKG_DEPS}
)

###Library
add_library (rug_simulated_user_functionality src/rug_simulated_user_functionality.cpp)
target_link_libraries(rug_simulated_user_functionality ${catkin_LIBRARIES})
target_link_libraries(rug_simulated_user_functionality ${PCL_LIBRARIES} ${OpenCV_LIBS} boost_system)
target_link_libraries(rug_simulated_user_functionality rug_kfold_cross_validation_functionality)


###Executable
add_executable(rug_simulated_user_hand_crafted_descriptor src/rug_simulated_user_hand_crafted_descriptor.cpp)
target_link_libraries(rug_simulated_user_hand_crafted_descriptor ${catkin_LIBRARIES} ${PCL_LIBRARIES} boost_system)
target_link_libraries(rug_simulated_user_hand_crafted_descriptor rug_simulated_user_functionality)

add_executable(rug_simulated_user_RGBD_deep_learning src/rug_simulated_user_RGBD_deep_learning.cpp)
target_link_libraries(rug_simulated_user_RGBD_deep_learning ${catkin_LIBRARIES} ${PCL_LIBRARIES} boost_system)
target_link_libraries(rug_simulated_user_RGBD_deep_learning rug_simulated_user_functionality)


