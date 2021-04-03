cmake_minimum_required(VERSION 3.0.2)
project(simulation_node_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  gazebo_ros
)

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    rospy
    roscpp
    geometry_msgs
    std_msgs
    gazebo_ros
)
