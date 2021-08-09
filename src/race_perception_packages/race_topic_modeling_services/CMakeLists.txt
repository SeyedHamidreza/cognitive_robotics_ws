cmake_minimum_required(VERSION 3.0.2)
project(race_topic_modeling_services)

set(MSG_DEPS
	roscpp
	std_msgs
	race_perception_msgs
)


set(PKG_DEPS
	nodelet
	${MSG_DEPS}
)



find_package(catkin REQUIRED COMPONENTS message_generation ${PKG_DEPS})


## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	topic_modelling.srv
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
#	INCLUDE_DIRS include 
#	LIBRARIES perception_db
	CATKIN_DEPENDS message_runtime ${PKG_DEPS}
)
