cmake_minimum_required(VERSION 3.8)
project(bizon_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(bizon_description REQUIRED)

set(dependencies
  rclcpp
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  bizon_description
)

add_library(bizon_ros2_control
  SHARED
  src/bizon_system_interface.cpp
)

target_include_directories(
  bizon_ros2_control
  PRIVATE
  include
)

ament_target_dependencies(
  bizon_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface bizon_ros2_control.xml)
install(TARGETS bizon_ros2_control DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

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

ament_package()
