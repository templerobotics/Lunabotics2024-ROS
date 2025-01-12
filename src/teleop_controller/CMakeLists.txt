cmake_minimum_required(VERSION 3.8)
project(teleop_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sparkcan REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)                  
find_package(rosidl_default_generators REQUIRED) 

# ifdef HARDWARE_ENABLED code
option(HARDWARE_ENABLED "Enable hardware support" OFF)
if(HARDWARE_ENABLED)
    add_definitions(-DHARDWARE_ENABLED)
endif()

# Include headers
include_directories(include)

# Generate interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetParameter.srv"
)

# Create nodes
add_executable(digging_belt_node src/DiggingBelt.cpp)
add_executable(digging_leadscrew_node src/DiggingLeadscrew.cpp)
add_executable(teleop_state_manager src/teleop_state_manager.cpp)
add_executable(drivebase_control src/DrivebaseControl.cpp)

# Add dependencies for digging_belt_node
ament_target_dependencies(digging_belt_node
  rclcpp
  sensor_msgs
  geometry_msgs
  sparkcan
  std_msgs
  std_srvs
)

# Add dependencies for digging_leadscrew_node
ament_target_dependencies(digging_leadscrew_node
  rclcpp
  sensor_msgs
  geometry_msgs
  sparkcan
  std_msgs
  std_srvs
)

# Add dependencies for teleop_state_manager
ament_target_dependencies(teleop_state_manager 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)

# Add dependencies for drivebase_control
ament_target_dependencies(drivebase_control 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)

# Link against generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(digging_belt_node "${cpp_typesupport_target}")
target_link_libraries(digging_leadscrew_node "${cpp_typesupport_target}")
target_link_libraries(teleop_state_manager "${cpp_typesupport_target}")
target_link_libraries(drivebase_control "${cpp_typesupport_target}")

# Install all nodes
install(TARGETS
  digging_belt_node
  digging_leadscrew_node
  teleop_state_manager
  drivebase_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()