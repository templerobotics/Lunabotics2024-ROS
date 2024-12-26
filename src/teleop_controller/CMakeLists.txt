cmake_minimum_required(VERSION 3.8)
project(teleop_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sparkcan REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(teleop_state_manager src/teleop_state_manager.cpp)
ament_target_dependencies(teleop_state_manager rclcpp sensor_msgs geometry_msgs sparkcan std_msgs)
include_directories(include)
install(TARGETS
  teleop_state_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
