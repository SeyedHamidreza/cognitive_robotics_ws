cmake_minimum_required(VERSION 3.0.2)
project(colormap_utils)


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if (PCL_FOUND)
else (PCL_FOUND)
	MESSAGE("PCL 1.7 not found. You must execute the following command\nsudo apt-get install libpcl-1.7-all-dev\n")
endif (PCL_FOUND)

set(LIBS ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Boost_LIBRARIES} )

set(PKG_DEPS
	roscpp
	std_msgs
	sensor_msgs
	rviz
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include 
	${catkin_INCLUDE_DIRS}	
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
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS ${PKG_DEPS}
)



add_library(${PROJECT_NAME} src/colormap_utils.cpp)
target_link_libraries(colormap_utils  ${LIBS})
