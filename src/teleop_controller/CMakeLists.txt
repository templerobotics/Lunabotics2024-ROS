cmake_minimum_required(VERSION 3.8)
project(teleop_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Set OpenGL preference to GLVND (modern OpenGL)
set(OpenGL_GL_PREFERENCE GLVND)

# Paths to external dependencies
set(GLFW_DIR "${CMAKE_SOURCE_DIR}/glfw")
set(GL3W_DIR "${CMAKE_SOURCE_DIR}/gl3w")
set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/imgui")
set(IMPLOT_DIR "${CMAKE_SOURCE_DIR}/implot")

# ----------------- GLFW -----------------
set(GLFW_BUILD_DOCS OFF CACHE BOOL "GLFW: Disable docs" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "GLFW: Disable tests" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "GLFW: Disable examples" FORCE)

add_subdirectory(${GLFW_DIR} EXCLUDE_FROM_ALL)
include_directories(${GLFW_DIR}/include)

# ----------------- GL3W -----------------
add_library(gl3w STATIC ${GL3W_DIR}/src/gl3w.c)
target_include_directories(gl3w PUBLIC ${GL3W_DIR}/include)

# ----------------- ImGui -----------------
include_directories(${IMGUI_DIR})
set(IMGUI_SOURCES 
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp 
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp)
add_library(imgui STATIC ${IMGUI_SOURCES})
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/backends)

# ----------------- ImPlot -----------------
include_directories(${IMPLOT_DIR})
set(IMPLOT_SOURCES 
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_items.cpp)
add_library(implot STATIC ${IMPLOT_SOURCES})
target_include_directories(implot PUBLIC ${IMPLOT_DIR})

# ----------------- ROS 2 Dependencies -----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sparkcan REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenGL REQUIRED)

# Include directories
include_directories(include)

# Generate interfaces first (service definitions)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetParameter.srv"
  "srv/SwitchMode.srv"
)

# ----------------- Add Executables -----------------
add_executable(drivebase_control src/DrivebaseControl.cpp)
add_executable(robot_mode src/ActivateMode.cpp)
add_executable(digging src/Digging.cpp)
add_executable(dashboard_viz src/DashboardViz.cpp)

# ----------------- Link Libraries -----------------
ament_target_dependencies(drivebase_control 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)

ament_target_dependencies(robot_mode 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)

ament_target_dependencies(digging 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  sparkcan 
  std_msgs
  std_srvs
)

ament_target_dependencies(dashboard_viz 
  rclcpp
  sensor_msgs
  std_msgs
)

# Fix missing dependencies for ImGui, ImPlot, and GLFW
target_link_libraries(dashboard_viz 
  glfw
  gl3w
  imgui
  implot
  OpenGL
)

# Link against generated service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(drivebase_control "${cpp_typesupport_target}")
target_link_libraries(robot_mode "${cpp_typesupport_target}")
target_link_libraries(digging "${cpp_typesupport_target}")
target_link_libraries(dashboard_viz "${cpp_typesupport_target}")

# Install nodes & header files
install(TARGETS
  drivebase_control
  robot_mode
  digging
  dashboard_viz
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

ament_package()
