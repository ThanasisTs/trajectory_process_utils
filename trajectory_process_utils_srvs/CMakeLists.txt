cmake_minimum_required(VERSION 2.8.3)
project(trajectory_process_utils_srvs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  Smoothing.srv
  Filtering.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_process_utils_srvs
 CATKIN_DEPENDS message_generation roscpp rospy 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


