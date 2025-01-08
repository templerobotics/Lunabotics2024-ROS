#!/bin/bash
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Print status message in white text on a blue background
print_status() {
    echo -e "\033[37;44m$1\033[0m"
}

print_status "Starting installation for Temple Robotics Lunabotics 2024..."
echo "WORKSPACE_DIR: ${WORKSPACE_DIR}"
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read -r REPLY

install_ros2_humble() {
    print_status "Installing ROS2 Humble"
    sudo apt-get update
    sudo apt-get install -y software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt-get update
    sudo apt-get install -y \
        ros-humble-desktop \
        python3-rosdep \
        python3-colcon-common-extensions \
        ros-humble-xacro \
        ros-humble-urdf \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher-gui \
        ros-humble-controller-manager \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-gazebo-ros2-control
}

install_system_dependencies() {
    print_status "Installing system and testing dependencies"
    sudo apt-get install -y \
        python3-rosdep \
        python3-colcon-common-extensions \
        python3-vcstool \
        build-essential \
        libgtest-dev \
        libgmock-dev \
        python3-pytest
}

install_sparkcan() {
    print_status "Adding Repository and Installing sparkcan"
    sudo add-apt-repository ppa:graysonarendt/sparkcan -y
    sudo apt update
    sudo apt install sparkcan -y
}

install_ros_dependencies() {
    print_status "Installing ROS Dependencies Using rosdep"
    cd "${WORKSPACE_DIR}"
    sudo rosdep init 2>/dev/null || true
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
}

setup_environment() {
    print_status "Setting up ROS environment"
    
    # Add ROS setup to bashrc if not already there
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
    
    # Add ROS workspace variable and sourcing
    if ! grep -q "ROS_WS=~/robotics/Lunabotics2024-ROS" ~/.bashrc; then
        echo "ROS_WS=~/robotics/Lunabotics2024-ROS" >> ~/.bashrc
        echo 'if [ -f "$ROS_WS/install/setup.bash" ]; then' >> ~/.bashrc
        echo '    source "$ROS_WS/install/setup.bash"' >> ~/.bashrc
        echo '    export AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH:$ROS_WS/install' >> ~/.bashrc
        echo '    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$ROS_WS/install' >> ~/.bashrc
        echo 'else' >> ~/.bashrc
        echo '    echo "ROS workspace not built. Run '\''colcon build'\'' in $ROS_WS"' >> ~/.bashrc
        echo 'fi' >> ~/.bashrc
    fi
    
    # Add build aliases
    if ! grep -q "alias build_hw=" ~/.bashrc; then
        echo "# Temple Robotics build aliases" >> ~/.bashrc
        echo "alias build_hw='colcon build --packages-select teleop_controller --cmake-args -DHARDWARE_ENABLED=ON'" >> ~/.bashrc
        echo "alias build_test='colcon build --packages-select teleop_controller --cmake-args -DHARDWARE_ENABLED=OFF'" >> ~/.bashrc
        echo "alias run_tests='colcon test --packages-select robot_testing --event-handlers console_direct+ --return-code-on-test-failure'" >> ~/.bashrc
        echo "alias build_and_test='build_test && run_tests'" >> ~/.bashrc
        echo "alias build_hw_all='build_hw && colcon build'" >> ~/.bashrc
    fi
}

main() {
    print_status "Updating system packages"
    sudo apt update && sudo apt upgrade -y
    
    install_ros2_humble
    install_system_dependencies
    install_sparkcan
    install_ros_dependencies
    setup_environment
    
    print_status "Installation complete! Please run: source ~/.bashrc"
}

main