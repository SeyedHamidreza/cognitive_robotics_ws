cmake_minimum_required(VERSION 2.8.3)
project(race_perception_utils)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system)
include_directories(${Boost_INCLUDE_DIRS})

#Find Graphviz
find_path(GRAPHVIZ_INCLUDE_DIR graphviz/gvc.h)
find_library(GRAPHVIZ_GVC_LIBRARY NAMES gvc )
find_package(GTK2 2.6 REQUIRED gtk)


if(${GRAPHVIZ_INCLUDE_DIR} STREQUAL GRAPHVIZ_INCLUDE_DIR-NOTFOUND )
	message(WARNING "Graphviz is not installed please do: sudo apt-get
			install libgraphviz-dev")
endif()

if(${GRAPHVIZ_GVC_LIBRARY} STREQUAL GRAPHVIZ_GVC_LIBRARY-NOTFOUND )
	message(WARNING "GVC library is not installed please do: sudo apt-get
			install libgraphviz-dev")
endif()

if(NOT GTK2_FOUND)
	message(WARNING "Gtk library is not installed please do: sudo apt-get
			install libgtk2.0-dev")
endif()


set(MSG_DEPS
	std_msgs
	actionlib_msgs
	sensor_msgs
	geometry_msgs
)


set(PKG_DEPS
	roscpp
	tf
	tf2_ros
	tf2_eigen
	pcl_conversions
	race_perception_msgs
	bondcpp
	actionlib
	nodelet
	${MSG_DEPS}
)

find_package(catkin REQUIRED COMPONENTS message_generation ${PKG_DEPS})

## Generate messages in the 'msg' folder
add_message_files(
	DIRECTORY msg
	FILES
	CallbackTime.msg
	Cycle.msg
	MeasureCycle.msg
	MeasurePDB.msg
)

#set(SRV_FILES
#)

#set(ACT_FILES
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES 
	${MSG_DEPS}
)


#catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})

catkin_package(
	INCLUDE_DIRS include
#	LIBRARIES ${PROJECT_NAME} print cycle tf_wrapper
	CATKIN_DEPENDS message_runtime ${PKG_DEPS}
)


SET(XGTK_PLUGIN gvplugin_xgtk)
include_directories(${GTK2_INCLUDE_DIRS})
include_directories(${GRAPHVIZ_INCLUDE_DIR})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

include_directories( include ${catkin_INCLUDE_DIRS})

#Find PCL 1.8
find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

if (NOT PCL_FOUND)
	MESSAGE("PCL 1.8 not found. You must execute the following command\nsudo apt-get install libpcl-1.8-all-dev\n")
endif (NOT PCL_FOUND)




# Catkin : You should reorder you cmakelists so that the calls to find_package(catkin ...), catkin_package(...) and include_directories() come before any calls to add_executable(), add_library() or target_link_libraries().

#catkin_package(
#    LIBRARIES ${PROJECT_NAME}
#    CATKIN_DEPENDS roscpp message_runtime geometry_msgs std_msgs
#    INCLUDE_DIRS include
#)


set(LIBS ${catkin_LIBRARIES} ${OpenCV_LIBS} ${EIGEN3_LIBS} ${Boost_LIBRARIES})



#Libraries
add_library(gvplugin_xgtk src/gvplugin_xgtk.cpp)
target_link_libraries(gvplugin_xgtk ${catkin_LIBRARIES})
add_dependencies(gvplugin_xgtk  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(callbacks_mht src/callbacks_mht.cpp)
target_link_libraries(callbacks_mht  ${catkin_LIBRARIES})
add_dependencies(callbacks_mht  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(race_perception_utils_print src/print.cpp)
target_link_libraries(race_perception_utils_print ${catkin_LIBRARIES})
add_dependencies(race_perception_utils_print  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(race_perception_utils_cycle src/cycle.cpp)
target_link_libraries(race_perception_utils_cycle  ${catkin_LIBRARIES})
add_dependencies(race_perception_utils_cycle  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(race_perception_utils_tf_wrapper src/tf_wrapper.cpp)
target_link_libraries(race_perception_utils_tf_wrapper ${catkin_LIBRARIES})
add_dependencies(race_perception_utils_tf_wrapper  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

###Executable
add_executable(test_print src/test_print.cpp)
target_link_libraries(test_print  ${LIBS} ${catkin_LIBRARIES})
add_dependencies(test_print ${PROJECT_NAME}_generate_messages_cpp)

add_executable(measure_cycle src/measure_cycle.cpp)
target_link_libraries(measure_cycle  ${LIBS} ${catkin_LIBRARIES})
add_dependencies(measure_cycle ${PROJECT_NAME}_generate_messages_cpp)

add_executable(kinect_static_tf src/kinect_static_tf.cpp)
target_link_libraries(kinect_static_tf  ${LIBS} ${catkin_LIBRARIES})
add_dependencies(kinect_static_tf  ${PROJECT_NAME}_generate_messages_cpp)

add_executable(test_wait_for_tf src/test_wait_for_tf.cpp)
target_link_libraries(test_wait_for_tf  ${LIBS} ${catkin_LIBRARIES})
add_dependencies(test_wait_for_tf ${PROJECT_NAME}_generate_messages_cpp)

#add_executable(point_head src/point_head.cpp)
#target_link_libraries(point_head  ${LIBS})

#add_executable(Y2D4 src/Y2D4.cpp)
#target_link_libraries(Y2D4  ${LIBS})



#############
## Testing ##
#############

## Test for correct C++ source code
#roslint_cpp()

#############
## Install ##
#############

## Install libraries
#install(TARGETS ${PROJECT_NAME}
#  	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

## Install header files
#install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
#install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executables
#install(TARGETS ${PROJECT_NAME}_demo
#	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#	)
