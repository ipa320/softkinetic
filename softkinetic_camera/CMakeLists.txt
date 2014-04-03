cmake_minimum_required(VERSION 2.8.3)
project(softkinetic_camera)

if(NOT EXISTS "/opt/softkinetic/DepthSenseSDK/include/DepthSense.hxx")
 message(WARNING "+++++++++ Did not find DepthSense.hxx +++++++++")
 message(WARNING "+++++++++ Please install DepthSenseSDK, see README.txt +++++++++")  
 return()
else()
  message(STATUS "+++++++++ Found DepthSense.hxx +++++++++")
endif()

find_package(catkin REQUIRED COMPONENTS pcl_ros cv_bridge roscpp rospy std_msgs image_transport message_generation)

#######################################
## Declare ROS messages and services ##
#######################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS pcl_ros roscpp rospy std_msgs message_runtime
)
###########
## Build ##
###########
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(/opt/softkinetic/DepthSenseSDK/include)
include_directories(/usr/include/suitesparse)

link_directories(/opt/softkinetic/DepthSenseSDK/lib)

add_executable(softkinetic_bringup_node src/softkinetic_start.cpp)
target_link_libraries(softkinetic_bringup_node DepthSense ${catkin_LIBRARIES})

