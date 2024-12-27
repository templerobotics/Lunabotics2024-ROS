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
add_executable(teleop_drivebase src/teleop_drivebase.cpp)


ament_target_dependencies(teleop_state_manager rclcpp sensor_msgs geometry_msgs sparkcan std_msgs)
ament_target_dependencies(teleop_drivebase rclcpp sensor_msgs geometry_msgs sparkcan std_msgs)

include_directories(include)


install(TARGETS
  teleop_state_manager
  teleop_drivebase
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
