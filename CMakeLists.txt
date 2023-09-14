cmake_minimum_required(VERSION 3.0.2)
project(mach1_hardware)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)