cmake_minimum_required(VERSION 3.5)
project(trx)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#-- Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RunScript.srv"
)

ament_export_dependencies(rosidl_default_runtime)

add_library(seed_command
  src/seed_command.cpp
)
set_target_properties(seed_command PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(seed_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(seed_hardware_interface SHARED src/seed_hardware_interface.cpp)

rosidl_target_interfaces(seed_hardware_interface
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(test_script
  src/test_script.cpp
)

target_link_libraries(test_script
  seed_command
)

ament_target_dependencies(test_script
  rclcpp
)


add_executable(test_move
  src/test_move.cpp
)

add_executable(rear_steering_node
  src/rear_steering_node.cpp
)

ament_target_dependencies(rear_steering_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(rear_steering_node
  seed_command
)

target_link_libraries(rear_steering_node
  seed_command
)

target_link_libraries(test_move
  seed_command
)

ament_target_dependencies(test_move
  rclcpp
)


ament_target_dependencies(seed_hardware_interface
  rclcpp
  std_msgs
  trajectory_msgs
  hardware_interface
   pluginlib
)
target_link_libraries(seed_hardware_interface
  seed_command
)

# ライブラリ
install(TARGETS
  seed_command
  seed_hardware_interface
  DESTINATION lib
)

# 実行ファイル
install(TARGETS
  test_script
  test_move
  rear_steering_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(FILES seed_hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface seed_hardware_interface_plugin.xml)
install(FILES trx.rviz
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
