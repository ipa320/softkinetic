cmake_minimum_required(VERSION 2.8.3)
project(softkinetic_camera)

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/common/catkin_ws/src/creative_interactive_gesture_camera/")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/g2o/cmake_modules/")
find_package(catkin REQUIRED COMPONENTS pcl_ros cv_bridge roscpp rospy std_msgs image_transport message_generation Eigen)
#find_package(Cholmod REQUIRED)
#FIND_PACKAGE(MRPT REQUIRED base)
#message("-- MRPT FOUND: " ${MRPT_FOUND})
#message("-- MRPT_LIBRARIES: " ${MRPT_LIBRARIES}) 
#find_package(G2O REQUIRED)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
# add_message_files(
#  FILES
#  floatTest.msg
#  Point2D.msg
#  Mapping2D.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
#)

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

# G2O library
# Set up the top-level include directories
#SET( G2O_INCLUDE ${PROJECT_SOURCE_DIR}/../g2o CACHE PATH "Directory of G2O")
#INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${G2O_INCLUDE})

# Add g2o lib dir
#SET(G2O_LIB ${G2O_INCLUDE}/lib)
#LINK_DIRECTORIES(${G2O_LIB})

#INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR}) #You can use CPARSE or CHOLMOD
#INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR}) 

#set(G2O_DIR ${G2O_INCLUDE}/g2o)
#include_directories(${G2O_DIR})
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#file(WRITE "test.txt" ${MRPT_INCLUDE_DIRS})

include_directories(/opt/softkinetic/DepthSenseSDK/include)
include_directories(/usr/include/suitesparse)
#include_directories(${MRPT_INCLUDE_DIRS})
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/core)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/stuff)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/examples)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/types/data)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/types/icp)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/types/sba)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/types/sim3)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/types/sclam2d)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/types/slam2d)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/types/slam3d)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/solvers/cholmod)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/solvers/csparse)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/solvers/dense)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/solvers/eigen)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/solvers/pcg)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/solvers/slam2d_linear)
#include_directories(${CMAKE_SOURCE_DIR}/g2o/g2o/solvers/structure_only)

link_directories(/opt/softkinetic/DepthSenseSDK/lib)

add_executable(softkinetic_bringup_node src/softkinetic_start.cpp)
target_link_libraries(softkinetic_bringup_node DepthSense ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS creative_interactive_gesture_camera creative_interactive_gesture_camera_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_creative_interactive_gesture_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
