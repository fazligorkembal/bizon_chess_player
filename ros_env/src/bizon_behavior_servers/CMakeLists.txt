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
)

pluginlib_export_plugin_description_file(bizon_core behavior_plugin.xml)

add_library(bizon_wait_behavior SHARED plugins/wait_plugin.cpp)
ament_target_dependencies(bizon_wait_behavior ${dependencies})

add_library(bizon_arm_behavior SHARED plugins/arm_plugin.cpp)
ament_target_dependencies(bizon_arm_behavior ${dependencies})

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

install(TARGETS ${library_name}
  bizon_wait_behavior
  bizon_board_behavior
  bizon_arm_behavior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name} 
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

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_libraries(${library_name}
  bizon_wait_behavior
  bizon_board_behavior
  bizon_arm_behavior
)
ament_export_dependencies(${dependencies})
ament_package()