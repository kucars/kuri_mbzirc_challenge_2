cmake_minimum_required(VERSION 2.8.3)
project(kuri_mbzirc_challenge_2_detecting_panel)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  tf_conversions
  eigen_conversions
  nav_msgs
  shape_msgs
  std_msgs
  rviz_visual_tools
  )

find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})
