cmake_minimum_required(VERSION 3.0.2)
project(mach1_hardware VERSION 1.0.0)

# Build and install third party libraries
add_subdirectory(external)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  ros_utils
  sensor_msgs
  std_msgs
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
)

include_directories(
  include
  external/camera_driver/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_server_node src/camera_server_node.cpp)
add_executable(camera_client_node src/camera_client_node.cpp)

target_link_libraries(camera_server_node camera_driver ${catkin_LIBRARIES})
target_link_libraries(camera_client_node camera_driver ${catkin_LIBRARIES})