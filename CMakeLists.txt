cmake_minimum_required(VERSION 3.8)
project(car2_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ignition-math6 REQUIRED)

# Include directories
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Plugin library
add_library(gazebo_ros_wheels_steerable SHARED
  src/gazebo_ros_wheels_steerable.cpp
)

ament_target_dependencies(gazebo_ros_wheels_steerable
  rclcpp
  gazebo_ros
  geometry_msgs
  nav_msgs
  std_msgs
  control_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ignition-math6
)

target_link_libraries(gazebo_ros_wheels_steerable
  ${GAZEBO_LIBRARIES}
)

# Install plugin library
install(
  TARGETS gazebo_ros_wheels_steerable
  LIBRARY DESTINATION lib
)

# Install models
install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(
  DIRECTORY cfg
  DESTINATION share/${PROJECT_NAME}
)

# Install worlds
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
