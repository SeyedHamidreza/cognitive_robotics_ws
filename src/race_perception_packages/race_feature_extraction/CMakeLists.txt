cmake_minimum_required(VERSION 3.0.2)
project(race_feature_extraction)

#Find CGAL
find_package(CGAL REQUIRED)
include( ${CGAL_USE_FILE} )
if (NOT CGAL_FOUND)
    MESSAGE("CGAL not found. Run #sudo apt-get install libcgal*\n")
endif (NOT CGAL_FOUND)


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (PCL_FOUND)
else (PCL_FOUND)
	MESSAGE("PCL 1.7 not found. You must execute the following command\nsudo apt-get install libpcl-1.7-all-dev\n")
endif (PCL_FOUND)


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
	#imperial_3d_object_tracking
	pcl_conversions
	race_perception_db
	race_perception_msgs
	race_perception_utils
)


find_package(catkin REQUIRED COMPONENTS message_generation ${PKG_DEPS})


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
	LIBRARIES feature_extraction spin_image
	CATKIN_DEPENDS message_runtime ${PKG_DEPS}
)




set(LIBS ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Boost_LIBRARIES} ${PCL_LIBRARIES})


#lib for low level functions for spin image computation and comparison
add_library(spin_image src/spin_image.cpp)
SET_TARGET_PROPERTIES(spin_image PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(spin_image ${PCL_LIBRARIES})
target_link_libraries(spin_image ${catkin_LIBRARIES})


# hybrid nodelet library feature extraction
add_library(feature_extraction src/feature_extraction.cpp)
SET_TARGET_PROPERTIES(feature_extraction PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(spin_image ${catkin_LIBRARIES})
target_link_libraries(feature_extraction ${PCL_LIBRARIES})
target_link_libraries(feature_extraction spin_image)
target_link_libraries(feature_extraction boost_system)

add_executable(feature_extraction_node src/feature_extraction_node.cpp)
SET_TARGET_PROPERTIES(feature_extraction_node PROPERTIES LINK_FLAGS -L${PCL_LIBRARY_DIRS})
target_link_libraries(spin_image ${catkin_LIBRARIES})
target_link_libraries(feature_extraction_node ${PCL_LIBRARIES})
target_link_libraries(feature_extraction_node boost_system)
target_link_libraries(feature_extraction_node feature_extraction)
target_link_libraries(feature_extraction_node  ${LIBS})
target_link_libraries (feature_extraction_node console_bridge)


