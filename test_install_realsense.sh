#!/bin/bash

# exit on any error
set -e

echo "Starting RealSense D455 Camera Installation..."

echo "Updating system packages..."
sudo apt update && sudo apt upgrade -y
if [ $? -eq 0 ]; then
    echo "System update successful"
else
    echo "System update failed"
    exit 1
fi

# install dependencies
echo "Installing dependencies..."
sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
if [ $? -eq 0 ]; then
    echo "Dependencies installation successful"
else
    echo "Dependencies installation failed"
    exit 1
fi

# install RealSense SDK
echo "Installing RealSense SDK..."
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true
sudo make uninstall && make clean
make -j$(nproc)  # Use all available CPU cores for faster compilation
if [ $? -eq 0 ]; then
    sudo make install
    echo "RealSense SDK installation successful"
else
    echo "RealSense SDK installation failed"
    exit 1
fi

# install ROS 2 Humble RealSense packages
echo "Installing ROS 2 Humble RealSense packages..."
sudo apt install -y ros-humble-realsense2-camera
sudo apt install -y ros-humble-realsense2-description
if [ $? -eq 0 ]; then
    echo "ROS 2 RealSense packages installation successful"
else
    echo "ROS 2 RealSense packages installation failed"
    exit 1
fi

echo "Installation completed successfully!"
echo "You can test the camera by running: ros2 launch realsense2_camera rs_launch.py"