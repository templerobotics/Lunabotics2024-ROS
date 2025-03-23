cmake_minimum_required(VERSION 3.10)
project(viz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/bin)

# Collect ImGui source files
set(IMGUI_SRC
    libs/imgui/imgui.cpp
    libs/imgui/imgui_draw.cpp
    libs/imgui/imgui_tables.cpp
    libs/imgui/imgui_widgets.cpp
    libs/imgui/backends/imgui_impl_glfw.cpp
    libs/imgui/backends/imgui_impl_opengl3.cpp
)

# Collect ImPlot source files (exclude headers)
set(IMPLOT_SRC
    libs/implot/implot.cpp  
    libs/implot/implot_items.cpp
)

# Find ROS 2 packages (e.g., rclcpp)
find_package(rclcpp REQUIRED)

# Add executable
add_executable(viz src/main.cpp src/gui.cpp libs/gl3w/src/gl3w.c ${IMGUI_SRC} ${IMPLOT_SRC} ../DrivebaseControl.cpp)

# Include directories
target_include_directories(viz PRIVATE
    libs/imgui
    libs/imgui/backends
    libs/gl3w/include
    libs/glfw/include
    libs/implot
    src  # Add src to the include directories for potential header files
    ..   # Add parent directory to the include path to access DrivebaseControl.cpp
    ../.. # Add two levels up from your current directory to access core.hpp
    ${rclcpp_INCLUDE_DIRS}  # Add ROS 2 rclcpp include directory
)

# Link ROS 2 rclcpp and other necessary libraries
target_link_libraries(viz PRIVATE
    rclcpp::rclcpp  # Link to ROS 2 rclcpp library
    glfw
    OpenGL::GL
    dl
    pthread
)

# Link the ROS 2 libraries for any other required ROS dependencies
ament_target_dependencies(viz rclcpp)
