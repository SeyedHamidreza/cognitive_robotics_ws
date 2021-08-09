cmake_minimum_required(VERSION 3.0.2)
project(race_object_descriptor)


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (NOT PCL_FOUND)
	MESSAGE("PCL 1.7 not found. You must execute the following command\nsudo apt-get install libpcl-1.7-all-dev\n")
endif (NOT PCL_FOUND)


find_package(Boost COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
find_package(OpenCV REQUIRED )
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


set(PKG_DEPS
	roscpp
	rospy
	roslib
	nodelet
	sensor_msgs
	pcl_conversions
	race_perception_db
	race_perception_msgs
	race_perception_utils
	colormap_utils
	nodelet
)


find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS} 

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
	LIBRARIES object_descriptor_functionality object_descriptor
	CATKIN_DEPENDS ${PKG_DEPS}
)



#Library
add_library(object_descriptor_functionality src/object_descriptor_functionality.cpp)
target_link_libraries(object_descriptor_functionality ${catkin_LIBRARIES} )
target_link_libraries(object_descriptor_functionality ${PCL_LIBRARIES})
target_link_libraries(object_descriptor_functionality ${OpenCV_LIBS}) 
target_link_libraries(object_descriptor_functionality boost_system)

###Hybrid nodelet library
add_library(object_descriptor src/object_descriptor.cpp)
target_link_libraries(object_descriptor ${catkin_LIBRARIES} )
target_link_libraries(object_descriptor ${PCL_LIBRARIES})
target_link_libraries(object_descriptor object_descriptor_functionality)
target_link_libraries(object_descriptor boost_system)

add_executable(object_descriptor_node src/object_descriptor_node.cpp)
target_link_libraries(object_descriptor_node ${catkin_LIBRARIES} )
target_link_libraries(object_descriptor_node ${PCL_LIBRARIES})
target_link_libraries(object_descriptor_node object_descriptor_functionality)
target_link_libraries(object_descriptor_node boost_system)
target_link_libraries(object_descriptor_node object_descriptor)
target_link_libraries (object_descriptor_node console_bridge)


