cmake_minimum_required(VERSION 3.10)
project(bizon_lifecycle_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

find_package(bizon_util REQUIRED)
find_package(bizon_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(bondcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

set(dependencies
  rclcpp
  std_srvs
  rclcpp_action
  diagnostic_updater
  bizon_util
  bizon_msgs
  bondcpp
  rclcpp_components
)

add_library(${library_name}
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

# bizon_behavior_clients (Task 6, F5) links LifecycleManagerClient into
# is_system_active_node, a BT plugin built as a SHARED library. A static
# archive built without -fPIC cannot be linked into a shared object (ld:
# "relocation ... can not be used when making a shared object; recompile
# with -fPIC"), so this must be position-independent even though it stays a
# static library itself.
set_target_properties(${library_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)

ament_target_dependencies(${library_name} "${dependencies}")

add_executable(lifecycle_manager_node src/main.cpp)
target_link_libraries(lifecycle_manager_node ${library_name})
ament_target_dependencies(lifecycle_manager_node "${dependencies}")


install(DIRECTORY include/
  DESTINATION include/
)

# ${library_name} (bizon_lifecycle_manager_core) must actually land in the
# install tree, not just get exported by name below: bizon_behavior_clients
# (Task 6, F5) is the first consumer of LifecycleManagerClient outside this
# package, and ament_export_libraries() alone only *declares* the library
# name for downstream find_package() -- it does not install the archive.
# Without this, downstream find_package(bizon_lifecycle_manager) fails at
# CMake configure time with "exports the library ... which couldn't be
# found".
install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_manager_node
  DESTINATION lib/${PROJECT_NAME}
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
