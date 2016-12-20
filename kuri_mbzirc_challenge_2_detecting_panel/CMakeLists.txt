cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2_detecting_panel)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  #eigen_conversions
  #geometry_msgs
  #nav_msgs
  pcl_ros
  roscpp
  #sensor_msgs
  #shape_msgs
  #std_msgs
  tf
  tf_conversions
  #rviz_visual_tools
  velodyne_driver
  velodyne_pointcloud
  velodyne_msgs
  )

#find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} pcl_conversions)

#find_package(PCL REQUIRED)
find_package(PCL 1.7 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS roscpp velodyne_driver velodyne_pointcloud
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})


add_executable(determine_wall src/determine_wall.cpp)
target_link_libraries(determine_wall ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(determine_wall ${catkin_EXPORTED_TARGETS})
