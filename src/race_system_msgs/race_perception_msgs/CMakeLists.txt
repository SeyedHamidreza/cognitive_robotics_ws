cmake_minimum_required(VERSION 3.0.2)
project(race_perception_msgs)

set(PKG_DEPS
	roscpp
	std_msgs
	actionlib_msgs
	sensor_msgs
	geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS
		  roscpp
		  message_generation
		  ${PKG_DEPS}
)

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES
	CompleteRTOV.msg
	ObjectCategoryBayesModel.msg
	RRTOV.msg
	TOVI.msg
	GTOV.msg
	ObjectCategory.msg
	RTOV.msg
	NOCD.msg
	PCTOV.msg
	SITOV.msg
)

## Generate services in the 'srv' folder
#add_service_files(
#	DIRECTORY srv
#	FILES
#)


## Generate services in the 'action' folder
#add_action_files(
#	DIRECTORY action
#	FILES
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES 
	${PKG_DEPS}
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  	INCLUDE_DIRS include
	#LIBRARIES ${PROJECT_NAME}
  	CATKIN_DEPENDS ${PKG_DEPS}
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



#############
## Install ##
#############


## Install project namespaced headers
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE)

#############
## Testing ##
#############

