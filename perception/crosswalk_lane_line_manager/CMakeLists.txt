cmake_minimum_required(VERSION 3.5)
project(crosswalk_lane_line_manager)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(aiformula_interfaces REQUIRED)
find_package(common_cpp REQUIRED)

add_executable(crosswalk_lane_line_manager
  src/crosswalk_lane_line_manager.cpp
  src/crosswalk_lane_line_manager_node.cpp
)

target_include_directories(crosswalk_lane_line_manager PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(crosswalk_lane_line_manager
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  PCL
  pcl_conversions
  aiformula_interfaces
  common_cpp
)

install(TARGETS
  crosswalk_lane_line_manager
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
