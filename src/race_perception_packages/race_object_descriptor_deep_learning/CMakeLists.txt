cmake_minimum_required(VERSION 3.0.2)
project(race_object_descriptor_deep_learning)

find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS})

find_package(PCL 1.7 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (PCL_FOUND)
else (PCL_FOUND)
	MESSAGE("PCL 1.7 not found. You must execute the following command\nsudo apt-get install libpcl-1.7-all-dev\n")
endif (PCL_FOUND)


set(PKG_DEPS
	roscpp
	std_msgs
	roslib
	nodelet
	sensor_msgs
	pcl_conversions
	race_perception_db
	race_perception_msgs
 	race_perception_utils
 	colormap_utils
 	race_deep_learning_feature_extraction
)


find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
	${catkin_INCLUDE_DIRS}
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
	LIBRARIES object_descriptor_deep_learning_functionality object_descriptor_deep_learning
	CATKIN_DEPENDS ${PKG_DEPS}
)


###Library 
add_library(object_descriptor_deep_learning_functionality src/object_descriptor_deep_learning_functionality.cpp)
target_link_libraries(object_descriptor_deep_learning_functionality ${catkin_LIBRARIES})
target_link_libraries(object_descriptor_deep_learning_functionality ${PCL_LIBRARIES})
target_link_libraries(object_descriptor_deep_learning_functionality boost_system)

#hybrid nodelet library
add_library(object_descriptor_deep_learning src/object_descriptor_deep_learning.cpp)
target_link_libraries(object_descriptor_deep_learning ${catkin_LIBRARIES})
target_link_libraries(object_descriptor_deep_learning ${PCL_LIBRARIES})
target_link_libraries(object_descriptor_deep_learning object_descriptor_deep_learning_functionality)
target_link_libraries(object_descriptor_deep_learning boost_system)

###Executable
add_executable(object_descriptor_deep_learning_node src/object_descriptor_deep_learning_node.cpp)
target_link_libraries(object_descriptor_deep_learning_node ${catkin_LIBRARIES})
target_link_libraries(object_descriptor_deep_learning_node ${PCL_LIBRARIES})
target_link_libraries(object_descriptor_deep_learning_node object_descriptor_deep_learning_functionality)
target_link_libraries(object_descriptor_deep_learning_node boost_system)
target_link_libraries (object_descriptor_deep_learning_node console_bridge)


