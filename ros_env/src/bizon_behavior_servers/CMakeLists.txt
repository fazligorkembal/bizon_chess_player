cmake_minimum_required(VERSION 3.8)
project(bizon_behavior_servers LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CUDA_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(bizon_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(bizon_core REQUIRED)
find_package(OpenCV 4.0 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(CUDA REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(bizon_msgs REQUIRED)
find_package(bizon_chess REQUIRED)
find_package(rcl_interfaces REQUIRED)

#################################################################
include_directories(
  ${CUDA_INCLUDE_DIRS}
  /usr/local/cuda/include
)
link_directories(/usr/local/cuda/lib64)

#################################################################



list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(
  include
)
set(library_name behavior_server_core)
set(executable_name behavior_server)

set(dependencies
  rclcpp
  rclcpp_components
  bizon_util
  pluginlib
  bizon_core
  OpenCV
  cv_bridge
  moveit_ros_planning_interface
  bizon_msgs
  bizon_chess
)

pluginlib_export_plugin_description_file(bizon_core behavior_plugin.xml)

# Process-wide game-session state (events.log, moves.txt path, error
# bundles), shared by every plugin. Deliberately its own small SHARED
# library rather than a singleton compiled into each plugin library: the
# four plugins below are separate .so files that pluginlib dlopen()s
# independently, so a Meyer's singleton duplicated into each of them would
# not actually be one object at runtime. Every plugin library links this one
# with ordinary DT_NEEDED linkage instead, which the dynamic loader
# deduplicates by soname regardless of dlopen() order -- see
# debug_session.hpp's class comment for the full reasoning.
add_library(bizon_debug_session SHARED src/debug_session.cpp)
ament_target_dependencies(bizon_debug_session ${dependencies})

add_library(bizon_wait_behavior SHARED plugins/wait_plugin.cpp)
ament_target_dependencies(bizon_wait_behavior ${dependencies})
target_link_libraries(bizon_wait_behavior bizon_debug_session)

add_library(bizon_arm_behavior SHARED plugins/arm_plugin.cpp)
ament_target_dependencies(bizon_arm_behavior ${dependencies})
target_link_libraries(bizon_arm_behavior bizon_debug_session)

add_library(bizon_decision_behavior SHARED plugins/decision_plugin.cpp src/stockfish_process.cpp)
ament_target_dependencies(bizon_decision_behavior ${dependencies})
target_link_libraries(bizon_decision_behavior bizon_debug_session)

# TensorRT source files
set(TENSORRT_SOURCES
  src/tensorrt/block.cpp
  src/tensorrt/model.cpp
  src/tensorrt/postprocess.cpp
  src/tensorrt/preprocess.cu
)

add_library(bizon_board_behavior SHARED
  plugins/board_plugin.cpp
  ${TENSORRT_SOURCES}
)
ament_target_dependencies(bizon_board_behavior ${dependencies})
target_link_libraries(bizon_board_behavior
  bizon_debug_session
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  cudart
  nvinfer
  nvinfer_plugin
)
set_target_properties(bizon_board_behavior PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON
)

add_library(${library_name} SHARED
  src/behavior_server.cpp
)
ament_target_dependencies(${library_name} ${dependencies})

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})
ament_target_dependencies(${executable_name} ${dependencies})
rclcpp_components_register_nodes(${library_name} "bizon_behavior_servers::BehaviorServer")

# Subscribes /rosout and appends WARN-and-above messages from this robot's
# namespace to <session>/rosout.log. A standalone rclcpp node, not a
# behavior plugin -- it has no goal to serve and nothing else needs to
# command it, so lifecycle-managing it alongside behavior_server would only
# add ceremony. Launched directly by bizon_lifecycle_dev.launch.py instead.
add_executable(rosout_logger src/rosout_logger_main.cpp)
target_link_libraries(rosout_logger bizon_debug_session)
ament_target_dependencies(rosout_logger rclcpp rcl_interfaces)

install(TARGETS ${library_name}
  bizon_debug_session
  bizon_wait_behavior
  bizon_board_behavior
  bizon_arm_behavior
  bizon_decision_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} rosout_logger
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
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
  find_package(tinyxml2_vendor REQUIRED)
  find_package(TinyXML2 REQUIRED)
  ament_add_gtest(test_behavior_plugin_xml test/test_behavior_plugin_xml.cpp)
  target_link_libraries(test_behavior_plugin_xml tinyxml2::tinyxml2)
  target_compile_definitions(test_behavior_plugin_xml PRIVATE
    BEHAVIOR_PLUGIN_XML_PATH="${CMAKE_CURRENT_SOURCE_DIR}/behavior_plugin.xml")

  ament_add_gtest(test_debug_session_format test/test_debug_session_format.cpp)
  target_include_directories(test_debug_session_format PRIVATE include)

  ament_add_gtest(test_stockfish_process test/test_stockfish_process.cpp src/stockfish_process.cpp)
  target_include_directories(test_stockfish_process PRIVATE include)
  # The `stockfish` binary this test execlp()s is built by the README's
  # install step into Stockfish/src at the repo root, never onto PATH (there
  # is no system-wide install step). The container mounts the repo at this
  # fixed path (see CLAUDE.md), so prepend it just for this test binary
  # rather than requiring every dev/CI shell to export PATH itself.
  set_tests_properties(test_stockfish_process PROPERTIES
    ENVIRONMENT "PATH=/home/user/Documents/bizon_chess_player/Stockfish/src:$ENV{PATH}")

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_libraries(${library_name}
  bizon_debug_session
  bizon_wait_behavior
  bizon_board_behavior
  bizon_arm_behavior
  bizon_decision_behavior
)
ament_export_dependencies(${dependencies})
ament_package()