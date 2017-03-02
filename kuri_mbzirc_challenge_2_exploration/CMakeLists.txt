cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2_exploration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib

  #husky_navigation
  #move_base
  kuri_mbzirc_challenge_2_msgs
  kuri_mbzirc_challenge_2_tools
  pcl_ros
  pcl_conversions
  sensor_msgs
  tf
  tf_conversions
  velodyne_driver
  velodyne_pointcloud
  velodyne_msgs
)


find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} )

#find_package(PCL REQUIRED)
find_package(PCL 1.7 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs tf velodyne_driver velodyne_pointcloud
  DEPENDS system_lib
)


#include_directories(${catkin_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
#Not the cleanest way, but the only way I could include header files from *_tools package
include_directories(../kuri_mbzirc_challenge_2_tools/include)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})



#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE)

add_library(pointcloud_gps_filter src/gps_conversion.cpp src/pointcloud_gps_filter.cpp)

add_executable(detection src/main.cpp src/detection.cpp src/panel_searching.cpp)
target_link_libraries(detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(detection ${catkin_EXPORTED_TARGETS})

add_executable(box_location src/box_location.cpp)
target_link_libraries(box_location ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(box_location ${catkin_EXPORTED_TARGETS})

add_executable(test_gps_filter_velodyne src/test_gps_filter_velodyne.cpp)
target_link_libraries(test_gps_filter_velodyne pointcloud_gps_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(test_gps_filter_velodyne ${catkin_EXPORTED_TARGETS})

add_executable(velodyne_box_detector src/velodyne_box_detector.cpp)
target_link_libraries(velodyne_box_detector pointcloud_gps_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(velodyne_box_detector ${catkin_EXPORTED_TARGETS})
