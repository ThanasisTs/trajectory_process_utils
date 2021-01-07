cmake_minimum_required(VERSION 3.0.2)
project(offline_trajectory_process)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  trajectory_custom_msgs
  trajectory_process_utils_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

