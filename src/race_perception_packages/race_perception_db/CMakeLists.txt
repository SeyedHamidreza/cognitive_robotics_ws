cmake_minimum_required(VERSION 3.0.2)
project(race_perception_db)


#find_library(UUID_LIB NAMES libuuid.so.1.3.0 PATHS "/usr/lib/x86_64-linux-gnu")
find_library(UUID_LIB NAMES libuuid.so PATHS "/lib/x86_64-linux-gnu")
if( UUID_LIB STREQUAL "UUID_LIB-NOTFOUND" )
    message( FATAL_ERROR "Could not find uuid library")
else( UUID_LIB STREQUAL "UUID_LIB-NOTFOUND" )
    message("Could find uuid library")
endif()

message(Debug: ${UUID_LIB})

find_library(LEVELDB_LIB NAMES libleveldb.so PATHS "/usr/lib" "/usr/local/lib")
message(Debug: ${LEVELDB_LIB})
if( LEVELDB_LIB STREQUAL "LEVELDB_LIB-NOTFOUND" )
    message( FATAL_ERROR "Could not find libleveldb.so installed in your system. Please install leveldb. Follow instructions at https://gist.github.com/mriem/7252054 to install")
endif()


find_path(LEVELDB_INCLUDE_DIR leveldb db.h PATHS "/usr/lib" "/usr/local/lib")
if( LEVELDB_INCLUDE_DIR STREQUAL "LEVELDB_INCLUDE_DIR-NOTFOUND" )
	message( FATAL_ERROR "Could not find leveldb/db.h installed in your system. Please install leveldb. Follow instructions at https://gist.github.com/mriem/7252054 to install")
endif()


set(MSG_DEPS
	roscpp
	std_msgs
	geometry_msgs
	sensor_msgs
	race_perception_msgs
	race_perception_utils
	actionlib
)


set(PKG_DEPS
	nodelet
	race_perception_utils
	${MSG_DEPS}
)


find_package(catkin REQUIRED COMPONENTS message_generation ${PKG_DEPS})

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES
	MsgTest.msg  
	Slice.msg
)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY srv
	FILES
	Delete.srv
	GetKeys.srv
	GetRTOVs.srv
	LastNodeID.srv
	GetAllObjectCat.srv
	GetPDBReport.srv
	GetSITOVs.srv
	Put.srv
	get_db_address.srv
	GetRTOV.srv
	Get.srv
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
	${LEVELDB_INCLUDE_DIR}
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
	LIBRARIES perception_db
	CATKIN_DEPENDS message_runtime ${PKG_DEPS}
)


add_library(perception_db src/perception_db.cpp)
target_link_libraries(perception_db ${LEVELDB_LIB} )
target_link_libraries(perception_db ${catkin_LIBRARIES})
add_dependencies(perception_db ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(perception_db ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(perception_db_node src/perception_db_node.cpp)
target_link_libraries(perception_db_node ${catkin_LIBRARIES})
target_link_libraries(perception_db_node perception_db ${LEVELDB_LIB} ${catkin_LIBRARIES})


add_executable(report_db_continuous src/report_db_continuous.cpp)
target_link_libraries(report_db_continuous ${catkin_LIBRARIES})
target_link_libraries(report_db_continuous perception_db ${catkin_LIBRARIES})

add_executable(pdb_tutorial src/pdb_tutorial.cpp)
target_link_libraries(pdb_tutorial ${catkin_LIBRARIES})
target_link_libraries(pdb_tutorial perception_db)

