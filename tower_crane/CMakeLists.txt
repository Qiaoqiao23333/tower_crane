# Use a modern CMake version required by ROS 2
cmake_minimum_required(VERSION 3.8)

# Project name should be lowercase to follow conventions
project(tower_crane)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the ament_cmake build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(canopen_core REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(canopen_base_driver REQUIRED)
find_package(canopen_proxy_driver REQUIRED)
find_package(lely_core_libraries REQUIRED)


# cogen_dcf(robot_control)  # Temporarily commented out - master.dcf should be generated or provided manually

# Install package resources (URDF, meshes, config, launch)
install(DIRECTORY
  launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES config/tower_crane_ros2_control.yaml
  DESTINATION share/${PROJECT_NAME}/config
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# This is the modern way to install resource files in ROS 2.
# It makes sure that launch files, models, etc., are placed
# in the 'install/tower_crane/share/tower_crane' directory
# so ROS 2 tools can find them.

# This registers the package with the ament index. It must be at the end.
ament_package()
