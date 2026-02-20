cmake_minimum_required(VERSION 3.8)
project(bizon_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(bizon_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(bondcpp REQUIRED)
find_package(bond REQUIRED)
find_package(action_msgs REQUIRED)


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  lifecycle_msgs
  bizon_msgs
  bondcpp
  bond
  action_msgs
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/node_thread.cpp
  src/node_utils.cpp
  src/lifecycle_node.cpp
  src/lifecycle_service_client.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(
  TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
