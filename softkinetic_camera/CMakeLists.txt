cmake_minimum_required(VERSION 2.8.3)
project(softkinetic_camera)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    pcl_ros
    cv_bridge
    image_transport
    camera_info_manager
    message_generation
    dynamic_reconfigure)

# find the DepthSense SDK
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
find_package(DepthSenseSDK)

if(NOT DepthSenseSDK_FOUND)
  message(FATAL_ERROR "+++++++++ Could not locate the DepthSense SDK +++++++++
Please install the SDK before trying to build the 'softkinetic_camera' package, see README.txt.

Cannot continue, aborting.")
  return()
endif()

generate_dynamic_reconfigure_options(cfg/Softkinetic.cfg)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs pcl_ros cv_bridge image_transport camera_info_manager message_runtime dynamic_reconfigure
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
add_dependencies(softkinetic_bringup_node ${PROJECT_NAME}_gencfg)
target_link_libraries(softkinetic_bringup_node ${DepthSenseSDK_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS softkinetic_bringup_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
