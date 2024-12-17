#!/bin/bash

# Print status message in white text on a blue background
print_status () {
  echo -e "\033[37;44m$1\033[0m"
}

# Function to add a line to a file if it doesn't exist
ainsl () {
  grep -qF -- "$1" "$2" || echo "$1" >> "$2"
}

# Print overview message
echo ""
echo "[Note] Target OS version  >>> Ubuntu 22.04.x (Jammy Jellyfish)"
echo "[Note] Target ROS version >>> ROS2 Humble"
echo "[Note] Project path       >>> $(pwd)"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read -r REPLY

# Set variables
name_os_version=${name_os_version:="jammy"}
name_ros_version=${name_ros_version:="humble"}

print_status "[Set up your sources.list and keys for ROS 2]"
sudo apt-get update -y && sudo apt-get install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${name_os_version} main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

print_status "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

print_status "[Install build environment and dependencies]"
sudo apt-get install -y build-essential python3-colcon-common-extensions python3-rosdep python3-vcstool git

print_status "[Install ROS2 Humble Desktop]"
sudo apt-get install -y ros-humble-desktop

print_status "[Install required ROS2 packages]"
sudo apt-get install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-core \
  ros-humble-nav-msgs \
  ros-humble-sensor-msgs \
  ros-humble-tf2-ros \
  ros-humble-teleop-twist-keyboard \
  ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-hardware-interface

print_status "[Installing some ROS2 Humble C++ Dependencies]"
sudo apt update && sudo apt install -y ros-humble-rclcpp ros-humble-ros-base ros-dev-tools



print_status "[Initialize rosdep and Update]"
if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

print_status "[Set up environment variables]"
ainsl "source /opt/ros/$name_ros_version/setup.bash" ~/.bashrc
ainsl "source $(pwd)/install/setup.bash" ~/.bashrc

source /opt/ros/$name_ros_version/setup.bash

print_status "[Build the workspace]"
if [ -d src ]; then
  colcon build
else
  echo "No 'src' folder found in the workspace. Please create it and add your ROS 2 packages."
fi

print_status "[Installation complete!]"
exit 0

