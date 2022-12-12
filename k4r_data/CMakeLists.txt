cmake_minimum_required(VERSION 2.8.3)
project(k4r_data)

find_package(catkin REQUIRED
  rospy
  std_msgs
  geometry_msgs
  k4r_data_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
  geometry_msgs
  k4r_data_msgs
)
