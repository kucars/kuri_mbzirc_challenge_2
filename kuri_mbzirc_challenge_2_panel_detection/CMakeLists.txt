cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2_panel_detection)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  #eigen_conversions
  kuri_mbzirc_challenge_2_msgs
  pcl_ros
  pcl_conversions
  roscpp
  tf
  tf_conversions
  laser_geometry
  )

#find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS})

#find_package(PCL REQUIRED)
find_package(PCL 1.7 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(circumnavigation_waypoints src/circumnavigation_waypoints.cpp)
target_link_libraries(circumnavigation_waypoints ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(circumnavigation_waypoints ${catkin_EXPORTED_TARGETS})

add_executable(velodyne_box_detector src/velodyne_box_detector.cpp)
target_link_libraries(velodyne_box_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(velodyne_box_detector ${catkin_EXPORTED_TARGETS})
