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

# Define executables
add_executable(teleop_controller src/teleop_controller.cpp)

# Set target dependencies
ament_target_dependencies(teleop_controller rclcpp sensor_msgs geometry_msgs sparkcan)

# Install targets
install(TARGETS
  teleop_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
