cmake_minimum_required(VERSION 3.8)
project(bizon_chess)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/board_geometry.cpp src/move_classification.cpp src/fen.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(DIRECTORY include/ DESTINATION include/)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_board_geometry test/test_board_geometry.cpp)
  target_link_libraries(test_board_geometry ${PROJECT_NAME})

  # Header-only: no ROS node, no Stockfish subprocess, nothing beyond gtest.
  ament_add_gtest(test_fen_utils test/test_fen_utils.cpp)

  ament_add_gtest(test_move_classification test/test_move_classification.cpp)
  target_link_libraries(test_move_classification ${PROJECT_NAME})

  ament_add_gtest(test_fen test/test_fen.cpp)
  target_link_libraries(test_fen ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
