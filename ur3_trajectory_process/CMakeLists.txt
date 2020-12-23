cmake_minimum_required(VERSION 3.0.2)
project(ur3_trajectory_process)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  cartesian_state_msgs
  trajectory_custom_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur3_trajectory_process
#  CATKIN_DEPENDS cartesian_state_msgs geometry_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_executable (ur3_trajectory_process src/ur3_trajectory_process.cpp src/utils.cpp)
target_link_libraries (ur3_trajectory_process ${catkin_LIBRARIES})
