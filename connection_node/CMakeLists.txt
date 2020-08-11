cmake_minimum_required(VERSION 2.8.3)
project(connection_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs roscpp rospy std_msgs
  INCLUDE_DIRS include# INCLUDE_DIRS: uncomment this if you package contains header files
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ./PracticalSocket
)

add_library(Host PracticalSocket/PracticalSocket.cpp)

add_executable(connection_node src/connection.cpp)
target_link_libraries(connection_node ${catkin_LIBRARIES} Host)