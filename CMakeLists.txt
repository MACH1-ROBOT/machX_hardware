cmake_minimum_required(VERSION 3.8)
project(machx_hardware VERSION 2.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

ament_package()

add_subdirectory(third_party)