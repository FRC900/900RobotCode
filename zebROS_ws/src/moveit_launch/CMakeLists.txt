cmake_minimum_required(VERSION 3.20)
project(moveit_launch)
include("../cmake_modules/CMakeOpt.cmake")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  field_obj
  geometry_msgs
  nav_msgs
  marker_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  apriltag_ros
)
