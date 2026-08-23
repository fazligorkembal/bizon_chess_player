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
find_package(bizon_msgs REQUIRED)
find_package(bizon_lifecycle_manager REQUIRED)
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
  bizon_msgs
  bizon_lifecycle_manager
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

add_library(decision_action_client_node SHARED plugins/action/decision_action_client_node.cpp)
list(APPEND plugin_libs decision_action_client_node)

add_library(run_until_success_node SHARED plugins/control/run_until_success_node.cpp)
list(APPEND plugin_libs run_until_success_node)

add_library(condition_node SHARED plugins/control/condition_node.cpp)
list(APPEND plugin_libs condition_node)

add_library(is_system_active_node SHARED plugins/condition/is_system_active_node.cpp)
list(APPEND plugin_libs is_system_active_node)

add_library(foreach_node SHARED plugins/control/foreach_node.cpp)
list(APPEND plugin_libs foreach_node)

add_library(recovery_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs recovery_node)

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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_recovery_node test/test_recovery_node.cpp)
  target_link_libraries(test_recovery_node recovery_node)
  ament_target_dependencies(test_recovery_node ${dependencies})

  ament_add_gtest(test_arm_action_client_node test/test_arm_action_client_node.cpp)
  target_link_libraries(test_arm_action_client_node arm_action_client_node)
  ament_target_dependencies(test_arm_action_client_node ${dependencies})

  ament_add_gtest(test_decision_action_client_node test/test_decision_action_client_node.cpp)
  target_link_libraries(test_decision_action_client_node decision_action_client_node)
  ament_target_dependencies(test_decision_action_client_node ${dependencies})

  # Pure C++, no ROS/BT dependency: covers PollThrottle's rate-limiting
  # decision in isolation, with synthetic timestamps instead of real sleeps.
  ament_add_gtest(test_poll_throttle test/test_poll_throttle.cpp)

  # Builds a real (small, standalone) BT::Tree to prove Task 6's E-stop
  # fixes behave, not just that the shipped XML has the right shape.
  ament_add_gtest(test_move_type_seeded_before_first_decision
    test/test_move_type_seeded_before_first_decision.cpp)
  target_link_libraries(test_move_type_seeded_before_first_decision condition_node)
  ament_target_dependencies(test_move_type_seeded_before_first_decision ${dependencies})

  ament_add_gtest(test_reactive_guard_halts_running_work
    test/test_reactive_guard_halts_running_work.cpp)
  ament_target_dependencies(test_reactive_guard_halts_running_work ${dependencies})

  # Structural guard only: parses the shipped chess_game.xml with tinyxml2 and
  # walks the DOM directly, without building a BT::Tree (which would require a
  # live MoveGroupInterface, a move_group action server, and every other BT
  # plugin's runtime dependencies -- unavailable in this unit-test
  # environment). See test/test_chess_game_tree_structure.cpp for what it does
  # and does not prove.
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  ament_add_gtest(test_chess_game_tree_structure test/test_chess_game_tree_structure.cpp)
  target_link_libraries(test_chess_game_tree_structure tinyxml2::tinyxml2)
  target_compile_definitions(test_chess_game_tree_structure PRIVATE
    CHESS_GAME_XML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees/chess_game.xml")

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