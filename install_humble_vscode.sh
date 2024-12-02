#!/bin/bash
 
# Print status message in white text on a blue background
print_status () {
  echo -e "\033[37;44m$1\033[0m"
}
 
# Function to add a line to a file if it doesn't exist
ainsl () {
  grep -qF -- "$1" $2 || echo "$1" >> $2
}
 
# Print message giving an overview of the installation process
echo ""
echo "[Note] Target OS version  >>> Ubuntu 22.04.x (Jammy Jellyfish)"
echo "[Note] Target ROS version >>> ROS2 Humble"
echo "[Note] ROS2 workspace     >>> $HOME/smartSensingIoT/robot_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read REPLY
 
print_status "[Set the target OS, ROS version, and name of ROS2 workspace]"
name_os_version=${name_os_version:="jammy"}
name_ros_version=${name_ros_version:="humble"}
name_ros2_workspace=${name_ros2_workspace:="robot_ws"}
 
# Ensure Python 3 is installed and set up pip
print_status "[Ensure Python 3.9 and pip are installed]"
sudo apt-get update -y
sudo apt-get install -y python3 python3-pip
 
# Upgrade pip to the latest version
python3 -m pip install --upgrade pip
 
# Ensure essential Python modules are installed
print_status "[Install essential Python modules]"
python3 -m pip install --upgrade netifaces pycryptodome
 
print_status "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y
 
print_status "[Install build environment, chrony, ntpdate, and curl]"
sudo apt-get install -y chrony ntpdate curl build-essential
sudo ntpdate ntp.ubuntu.com
 
print_status "[Add the ROS2 repository]"
if [ ! -e /etc/apt/sources.list.d/ros2-latest.list ]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
fi
 
print_status "[Download the ROS keys]"
roskey=$(apt-key list | grep "Open Robotics")
if [ -z "$roskey" ]; then
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi
 
print_status "[Check the ROS keys]"
roskey=$(apt-key list | grep "Open Robotics")
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborting installation]"
  exit 0
fi
 
print_status "[Update the package lists]"
sudo apt-get update -y
 
print_status "[Install ROS2 desktop and Gazebo]"
sudo apt-get install -y ros-$name_ros_version-desktop ros-$name_ros_version-gazebo-* 
 
print_status "[Install other ROS2 packages]"
sudo apt-get install -y \
  ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard \
  ros-humble-laser-proc \
  ros-humble-rgbd-launch \
  ros-humble-rosserial-arduino \
  ros-humble-rosserial-python \
  ros-humble-rosserial-client \
  ros-humble-rosserial-msgs \
  ros-humble-amcl \
  ros-humble-map-server \
  ros-humble-move-base \
  ros-humble-urdf \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-gmapping \
  ros-humble-navigation2 \
  ros-humble-interactive-markers
 
print_status "[Install ROS2 Turtlebot3 packages]"
sudo apt-get install -y \
  ros-humble-dynamixel-sdk \
  ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-simulations
 
print_status "[Install colcon and other tools]"
sudo apt-get install -y \
  python3-colcon-common-extensions \
  libcgal-dev \
  libarmadillo-dev \
  liblapack-dev
 
print_status "[Install additional Python modules]"
python3 -m pip install --upgrade \
  collision \
  networkx \
  numpy==1.17.4 \
  pandas==1.3 \
  protobuf==3.19.0 \
  scipy==1.8 \
  scikit-learn==1.3.2 \
  tensorboard==2.10.0 \
  tensorboardX==2.6 \
  tqdm \
  torch==2.2.1 \
  torchvision==0.17.1 \
  torchaudio==2.2.1
 
print_status "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.bash
sudo apt-get install -y \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool \
  build-essential
 
print_status "[Install rosdep]"
sudo apt-get install -y python3-rosdep
 
if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  print_status "[Initialize rosdep and Update]"
  sudo sh -c "rosdep init"
  rosdep update
fi
 
if ! [ -d $HOME/$name_ros2_workspace ]; then
  print_status "[Create the ROS2 workspace and build]"
  mkdir -p $HOME/$name_ros2_workspace/src
  cd $HOME/$name_ros2_workspace
  colcon build
fi
 
print_status "[Set the ROS2 environment]"
ainsl "alias eb='nano ~/.bashrc'" ~/.bashrc
ainsl "alias sb='source ~/.bashrc'" ~/.bashrc
ainsl "alias cw='cd ~/$name_ros2_workspace'" ~/.bashrc
ainsl "alias cs='cd ~/$name_ros2_workspace/src'" ~/.bashrc
ainsl "alias cb='cd ~/$name_ros2_workspace && colcon build'" ~/.bashrc
 
ainsl "source /opt/ros/$name_ros_version/setup.bash" ~/.bashrc
ainsl "source ~/$name_ros2_workspace/install/setup.bash" ~/.bashrc
 
ainsl "export ROS_DOMAIN_ID=0" ~/.bashrc
ainsl "export TURTLEBOT3_MODEL=burger" ~/.bashrc
 
# Install Visual Studio Code
print_status "[Install Visual Studio Code]"
 
# Add the Microsoft GPG key
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
rm -f packages.microsoft.gpg
 
# Add the VS Code repository
sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
 
# Update the package list and install VS Code
sudo apt-get update
sudo apt-get install -y code
 
# Verify the installation of VS Code and the 'code' command
if command -v code &> /dev/null
then
    print_status "Visual Studio Code installed successfully."
else
    echo "Failed to install Visual Studio Code. Aborting."
    exit 1
fi
 
# Install VS Code extensions
print_status "[Install VS Code Extensions]"
 
# List of extensions to install
extensions=(
    "ms-vscode.cpptools:C/C++"
    "twxs.cmake:CMake"
    "ms-vscode.cmake-tools:CMake Tools"
    "ms-python.vscode-pylance:Pylance"
    "ms-python.python:Python"
    "ms-iot.vscode-ros:ROS"
    "redhat.vscode-xml:XML"
  
)
 
# Loop through the list and install each extension
for ext in "${extensions[@]}"; do
    ext_id="${ext%%:*}"
    ext_name="${ext##*:}"
    print_status "Installing $ext_name extension"
    code --install-extension "$ext_id" --force
done
 
print_status "[Complete!!!]"
exit 0


 
