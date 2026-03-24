cmake_minimum_required(VERSION 3.8)
project(tower_crane_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(tower_crane_transmission_system SHARED
  src/tower_crane_transmission_system.cpp)

target_include_directories(tower_crane_transmission_system PUBLIC
  include)

ament_target_dependencies(tower_crane_transmission_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface
  tower_crane_hardware.xml)

install(TARGETS tower_crane_transmission_system
  DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(FILES tower_crane_hardware.xml
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)

ament_package()
