cmake_minimum_required(VERSION 2.8.3)
project(softkinetic_camera)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pcl_ros cv_bridge image_transport camera_info_manager)

# find the DepthSense SDK
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(DepthSenseSDK)

if(NOT DepthSenseSDK_FOUND)
  message(FATAL_ERROR "+++++++++ Could not locate the DepthSense SDK +++++++++
Please install the SDK before trying to build the 'softkinetic_camera' package, see README.txt.

Cannot continue, aborting.")
  return()
endif()


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
  CATKIN_DEPENDS roscpp std_msgs pcl_ros cv_bridge camera_info_manager
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${DepthSenseSDK_INCLUDE_DIRS}
)

add_executable(softkinetic_bringup_node src/softkinetic_start.cpp)
target_link_libraries(softkinetic_bringup_node ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS softkinetic_bringup_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
