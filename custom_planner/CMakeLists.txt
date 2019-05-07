cmake_minimum_required(VERSION 2.8.3)
project(custom_planner)

find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/custom_planner
)

add_library(custom_planner_lib src/custom_planner.cpp)