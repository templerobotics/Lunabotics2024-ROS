cmake_minimum_required(VERSION 3.8)
project(teleop_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# MACRO for hardware enabled. Precursor to implementing Gazebo simulation for our teleop robot. 
# If hardware was OFF, then sim would be used to control virtual robot. NEED to add simulation specific code for that to work properly as of Jan 22 2025

option(HARDWARE_ENABLED "Enable hardware support" ON)
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
add_executable(digging_belt src/DiggingBelt.cpp)
add_executable(digging_leadscrew src/DiggingLeadscrew.cpp)
add_executable(teleop_state_manager src/teleop_state_manager.cpp)
add_executable(drivebase_control src/DrivebaseControl.cpp)
add_executable(dumping src/Dumping.cpp)


ament_target_dependencies(digging_belt
  rclcpp
  sensor_msgs
  geometry_msgs
  sparkcan
  std_msgs
  std_srvs
)

ament_target_dependencies(digging_leadscrew
  rclcpp
  sensor_msgs
  geometry_msgs
  sparkcan
  std_msgs
  std_srvs
)

ament_target_dependencies(teleop_state_manager 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)

ament_target_dependencies(drivebase_control 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)

ament_target_dependencies(dumping 
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

target_link_libraries(digging_belt "${cpp_typesupport_target}")
target_link_libraries(digging_leadscrew "${cpp_typesupport_target}")
target_link_libraries(teleop_state_manager "${cpp_typesupport_target}")
target_link_libraries(drivebase_control "${cpp_typesupport_target}")
target_link_libraries(dumping "${cpp_typesupport_target}")


# install nodes
install(TARGETS
  digging_belt
  digging_leadscrew
  teleop_state_manager
  drivebase_control
  dumping
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()