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


include_directories(include)

# Generate interfaces first
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetParameter.srv"
)

add_executable(digging_leadscrew src/DiggingLeadscrew.cpp)
add_executable(drivebase_control src/DrivebaseControl.cpp)
add_executable(dumping src/Dumping.cpp)



ament_target_dependencies(digging_leadscrew
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

target_link_libraries(digging_leadscrew "${cpp_typesupport_target}")
target_link_libraries(drivebase_control "${cpp_typesupport_target}")
target_link_libraries(dumping "${cpp_typesupport_target}")


# install nodes & header files
install(TARGETS
  digging_leadscrew
  drivebase_control
  dumping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()