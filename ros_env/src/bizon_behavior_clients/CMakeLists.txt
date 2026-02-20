cmake_minimum_required(VERSION 3.8)
project(bizon_behavior_clients)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(library_name ${PROJECT_NAME})

include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(bizon_util REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
  rclcpp_action
  rclcpp_lifecycle
  builtin_interfaces
  behaviortree_cpp
  std_msgs
  sensor_msgs
  moveit_msgs
  moveit_ros_planning_interface
  bizon_util
  cv_bridge
)

add_executable(bizon_behavior_tree_client_main src/main.cpp)
target_link_libraries(bizon_behavior_tree_client_main ${library_name})

add_library(${library_name} SHARED src/behavior_tree_engine.cpp)
ament_target_dependencies(${library_name} ${dependencies})

#add_library(arm_action_client_node SHARED plugins/action/arm_action_client_node.cpp)
#list(APPEND plugin_libs arm_action_client_node)

add_library(wait_action_client_node SHARED plugins/action/wait_action_client_node.cpp)
list(APPEND plugin_libs wait_action_client_node)

add_library(arm_action_client_node SHARED plugins/action/arm_action_client_node.cpp)
list(APPEND plugin_libs arm_action_client_node)

add_library(board_action_isaac_client_node SHARED plugins/action/board_action_isaac_client_node.cpp)
target_link_libraries(board_action_isaac_client_node ${OpenCV_LIBS})
ament_target_dependencies(board_action_isaac_client_node ${dependencies})
list(APPEND plugin_libs board_action_isaac_client_node)

add_library(make_decision_client_node SHARED plugins/action/make_decision_client_node.cpp)
list(APPEND plugin_libs make_decision_client_node)

add_library(run_until_success_node SHARED plugins/control/run_until_success_node.cpp)
list(APPEND plugin_libs run_until_success_node)

add_library(condition_node SHARED plugins/control/condition_node.cpp)
list(APPEND plugin_libs condition_node)

add_library(foreach_node SHARED plugins/control/foreach_node.cpp)
list(APPEND plugin_libs foreach_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bizon_behavior_tree_client_main
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

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()