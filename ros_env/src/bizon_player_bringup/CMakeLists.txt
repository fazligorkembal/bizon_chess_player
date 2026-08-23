cmake_minimum_required(VERSION 3.8)
project(bizon_player_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
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

  # session_dir.py has no ROS import at all -- see its module docstring --
  # so this test needs nothing beyond a plain python3 + pytest, run here via
  # colcon test for convenience. It is equally runnable standalone:
  # `python3 launch/test_session_dir.py`.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_session_dir launch/test_session_dir.py)
endif()

ament_package()
