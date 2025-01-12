cmake_minimum_required(VERSION 3.8)
project(teleop_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sparkcan REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)                  
find_package(rosidl_default_generators REQUIRED) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetParameter.srv"
)

# ifdef HARDWARE_ENABLED code
option(HARDWARE_ENABLED "Enable hardware support" OFF)
if(HARDWARE_ENABLED)
    add_definitions(-DHARDWARE_ENABLED)
endif()

add_executable(teleop_state_manager src/teleop_state_manager.cpp)
add_executable(DrivebaseControl src/DrivebaseControl.cpp)

ament_target_dependencies(teleop_state_manager 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)

ament_target_dependencies(DrivebaseControl 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)




# Create library
add_library(${PROJECT_NAME}_lib
  src/DiggingLeadscrew.cpp
  src/MotorControllerGroup.cpp
)



# Link against generated service interfaces using the new method
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(teleop_state_manager "${cpp_typesupport_target}")
target_link_libraries(DrivebaseControl "${cpp_typesupport_target}")

include_directories(include)

install(TARGETS
  teleop_state_manager
  DrivebaseControl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()