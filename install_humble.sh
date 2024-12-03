#!/bin/bash

# Print status message in white text on a blue background
print_status () {
  echo -e "\033[37;44m$1\033[0m"
}

# Function to add a line to a file if it doesn't exist
ainsl () {
  grep -qF -- "$1" $2 || echo "$1" >> $2
}

# Print overview message
echo ""
echo "[Note] Target OS version  >>> Ubuntu 22.04.x (Jammy Jellyfish)"
echo "[Note] Target ROS version >>> ROS2 Humble"
echo "[Note] Project path       >>> $(pwd)"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read REPLY

print_status "[Set the target OS and ROS version]"
name_os_version=${name_os_version:="jammy"}
name_ros_version=${name_ros_version:="humble"}

print_status "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

print_status "[Install build environment and dependencies]"
sudo apt-get install -y build-essential curl python3-colcon-common-extensions

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

print_status "[Environment setup]"
source /opt/ros/$name_ros_version/setup.bash

print_status "[Install rosdep]"
sudo apt-get install -y python3-rosdep

if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  print_status "[Initialize rosdep and Update]"
  sudo sh -c "rosdep init"
  rosdep update
fi

# Get the workspace path
WORKSPACE_PATH=$(pwd)

print_status "[Clean up old build and install directories]"
rm -rf build install log

print_status "[Set the ROS2 environment]"
ainsl "source /opt/ros/$name_ros_version/setup.bash" ~/.bashrc
ainsl "source $WORKSPACE_PATH/install/setup.bash" ~/.bashrc

# Add useful aliases
ainsl "alias cw='cd $WORKSPACE_PATH'" ~/.bashrc
ainsl "alias cb='cd $WORKSPACE_PATH && colcon build'" ~/.bashrc

print_status "[Initial workspace build]"
colcon build

if [ $? -eq 0 ]; then
  print_status "[Build completed successfully!]"
else
  print_status "[Build failed. Check the errors above.]"
  exit 1
fi

print_status "[Complete!!!]"
echo "Please restart your terminal or run: source ~/.bashrc"
exit 0


