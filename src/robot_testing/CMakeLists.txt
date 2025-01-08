cmake_minimum_required(VERSION 3.20)
project(robot_testing)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(teleop_controller REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)


if(BUILD_TESTING)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)

  
  find_package(ament_cmake_gtest REQUIRED)
  find_package(GTest REQUIRED)
  
  include(GoogleTest)

  message(STATUS "Building ROS 2 teleop tests...")

  add_executable(teleop_tests
    tests_teleop/test_teleop_control.cpp
  )

  
  target_include_directories(teleop_tests PUBLIC
    ${CMAKE_SOURCE_DIR}/../teleop_controller/include
  )

  target_link_libraries(teleop_tests
    GTest::GTest
    GTest::Main
    gmock
    gmock_main
  )

  ament_target_dependencies(teleop_tests
    rclcpp
    teleop_controller
    std_msgs
    geometry_msgs
    sensor_msgs
    std_srvs
  )

  # Register tests
  gtest_discover_tests(teleop_tests)
endif()

ament_package()