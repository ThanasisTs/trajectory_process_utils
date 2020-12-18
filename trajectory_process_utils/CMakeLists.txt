cmake_minimum_required(VERSION 2.8.3)
project(trajectory_process_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  trajectory_custom_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
