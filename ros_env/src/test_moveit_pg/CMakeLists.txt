cmake_minimum_required(VERSION 3.8)
project(test_moveit_pg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED)
find_package(bizon_behavior_servers REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

#################################################################
include_directories(
  ${CUDA_INCLUDE_DIRS}
  /usr/local/cuda/include
)
link_directories(/usr/local/cuda/lib64)

#################################################################

set(dependencies
  rclcpp
  moveit_ros_planning_interface
  OpenCV
  bizon_behavior_servers
  sensor_msgs
  cv_bridge
)

add_executable(first_app src/first_app.cpp)
ament_target_dependencies(first_app "${dependencies}")

add_executable(pose_target src/pose_target.cpp)
ament_target_dependencies(pose_target "${dependencies}")

add_executable(pick_place_a8_to_h1 src/pick_place_a8_to_h1.cpp)
ament_target_dependencies(pick_place_a8_to_h1 "${dependencies}")

add_executable(preprocessing_calibration src/preprocessing_calibration.cpp)
ament_target_dependencies(preprocessing_calibration "${dependencies}")
target_link_libraries(preprocessing_calibration ${OpenCV_LIBS})

add_executable(benchmark_inference_pipeline src/benchmark_inference_pipeline.cpp)
ament_target_dependencies(benchmark_inference_pipeline "${dependencies}")
target_link_libraries(benchmark_inference_pipeline ${OpenCV_LIBS})

add_executable(board_action_dummy src/board_action_dummy.cpp)
ament_target_dependencies(board_action_dummy "${dependencies}")
target_link_libraries(board_action_dummy ${OpenCV_LIBS} nvinfer nvinfer_plugin cudart)

add_executable(test_param_executable src/test_param_executable.cpp)
ament_target_dependencies(test_param_executable rclcpp)

add_executable(test_stockfish_client src/test_stockfish_client.cpp)


#install(DIRECTORY include/ DESTINATION include)

install(TARGETS
  first_app
  pose_target
  pick_place_a8_to_h1
  preprocessing_calibration
  benchmark_inference_pipeline
  board_action_dummy
  test_stockfish_client
  test_param_executable
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
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

ament_package()
