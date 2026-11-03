cmake_minimum_required(VERSION 3.8)
project(bizon_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(serial_imu_node src/serial_imu_node.cpp)
target_include_directories(serial_imu_node PRIVATE include)
ament_target_dependencies(serial_imu_node rclcpp sensor_msgs)

install(TARGETS serial_imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_stream_line test/test_stream_line.cpp)
  target_include_directories(test_stream_line PRIVATE include)
endif()

ament_package()
