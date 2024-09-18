#!/bin/bash

set -xe

print_status () {
  echo -e "\033[37;44m$1\033[0m"
}

print_status "[Installing ROS2 Jazzy Jalisco...]"

# Install necessary packages
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y terminator locales software-properties-common curl python3-pip ccache

# Set locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

# Add ROS2 repository
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install ROS2 Jazzy
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros2-controllers ros-jazzy-xacro \
ros-jazzy-joint-state-publisher-gui ros-jazzy-turtlesim ros-jazzy-robot-localization ros-jazzy-joy \
ros-jazzy-joy-teleop ros-jazzy-tf-transformations ros-jazzy-plotjuggler ros-jazzy-plotjuggler-ros ros-jazzy-urdf-tutorial

# Install additional Python packages
sudo apt install python3-transforms3d

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Set the workspace path
workspace_path="/home/$USER/Lunabotics2024-ROS"
mkdir -p $workspace_path/src

print_status "[Configuring your bashrc to automatically source for ROS2]"

echo "alias uu='sudo apt update && sudo apt upgrade -y'" >> ~/.bashrc
echo "alias cb='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER_LAUNCHER=ccache --parallel-workers \$(nproc)'" >> ~/.bashrc

echo "# Source workspace automatically for every shell instance" >> ~/.bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

echo "# Cache previous compilation result for faster colcon builds" >> ~/.bashrc
echo 'export PATH="/usr/lib/ccache:$PATH"' >> ~/.bashrc

echo "if [ -f $workspace_path/install/setup.bash ]; then" >> ~/.bashrc
echo "    source $workspace_path/install/setup.bash" >> ~/.bashrc
echo "fi" >> ~/.bashrc

source ~/.bashrc

print_status "[Installing ros python network tables package forcefully in the LInux environment...]"
sudo pip3 install pynetworktables --break-system-packages


print_status "[Installing Visual Studio Code...]"
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg
sudo apt-get install -y apt-transport-https
sudo apt-get update
sudo apt-get install -y code

echo "Installing VS Code extensions..."

# List of extensions to install
extensions=(
    "ms-vscode.cpptools:C/C++"
    "twxs.cmake:CMake"
    "ms-vscode.cmake-tools:CMake Tools"
    "ms-python.vscode-pylance:Pylance"
    "ms-python.python:Python"
    "ms-iot.vscode-ros:ROS"
    "redhat.vscode-xml:XML"
    "ms-azuretools.vscode-docker:Docker"
)

# Loop through the list and install each extension
for ext in "${extensions[@]}"; do
    ext_id="${ext%%:*}"
    ext_name="${ext##*:}"
    echo "Installing $ext_name extension"
    code --install-extension "$ext_id" --force
done

# Test ROS2 installation with demo nodes
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 run demo_nodes_cpp talker"
gnome-terminal -- bash -c "source /opt/ros/jazzy/setup.bash; ros2 run demo_nodes_cpp listener"

print_status "[Successfully Installed ROS 2 Jazzy Jalisco, VS Code, Extensions, & Configured Bashrc. Exiting...]"

exit 0

