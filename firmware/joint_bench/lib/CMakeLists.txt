# Host build of lib/: the firmware's pure logic, compiled with the host compiler and tested
# with GoogleTest. The target build compiles the same sources from the CubeMX project's
# top-level CMakeLists.txt; this file is never used for the target.
#
# Sources and tests are globbed so that adding a module is adding its two files, with no
# edit here. CONFIGURE_DEPENDS makes a new file trigger a re-glob on the next build.
cmake_minimum_required(VERSION 3.22)
project(joint_bench_lib CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

file(GLOB JB_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB JB_TESTS CONFIGURE_DEPENDS tests/*.cpp)

enable_testing()
find_package(GTest REQUIRED)
add_executable(jb_tests ${JB_SOURCES} ${JB_TESTS})
target_include_directories(jb_tests PRIVATE include)
target_link_libraries(jb_tests PRIVATE GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(jb_tests)
