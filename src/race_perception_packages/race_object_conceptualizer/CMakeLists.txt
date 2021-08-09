cmake_minimum_required(VERSION 3.0.2)
project(race_object_conceptualizer)

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


set(MSG_DEPS
	roscpp
	std_msgs
	sensor_msgs
	race_perception_msgs
	race_perception_utils	
)


set(PKG_DEPS
	roslib
	nodelet
	race_feature_extraction
	${MSG_DEPS}
)


find_package(catkin REQUIRED COMPONENTS message_generation ${PKG_DEPS})


## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES
	UserObjectLabel.msg  
)


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES 
	${MSG_DEPS}
)


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
	LIBRARIES object_conceptualization object_conceptualizer
	CATKIN_DEPENDS message_runtime ${PKG_DEPS}
)


###Library
add_library(object_conceptualization src/object_conceptualization.cpp)
target_link_libraries(object_conceptualization ${catkin_LIBRARIES})
target_link_libraries(object_conceptualization boost_system)

###Hybrid library
add_library (object_conceptualizer src/object_conceptualizer.cpp)
target_link_libraries(object_conceptualizer object_conceptualization)
target_link_libraries(object_conceptualizer ${catkin_LIBRARIES})


###Executable 
add_executable(node_object_conceptualizer src/node_object_conceptualizer.cpp)
target_link_libraries(node_object_conceptualizer ${catkin_LIBRARIES})
target_link_libraries(node_object_conceptualizer boost_system)
target_link_libraries(node_object_conceptualizer object_conceptualization)


