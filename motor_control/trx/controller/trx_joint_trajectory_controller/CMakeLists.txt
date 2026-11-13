cmake_minimum_required(VERSION 3.16)
project(trx_joint_trajectory_controller)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
  control_msgs  
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(example_interfaces REQUIRED)

add_library(trx_joint_trajectory_controller SHARED
  src/joint_trajectory_controller.cpp
)

ament_target_dependencies(trx_joint_trajectory_controller
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  trajectory_msgs
  control_msgs
)

pluginlib_export_plugin_description_file(controller_interface trx_joint_trajectory_plugin.xml)

install(TARGETS
    trx_joint_trajectory_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
