# Repo for Temple University ROS2-based Autonomous Mining Robot for NASA Lunabotics Competition

## How to Utilize this repo:

### Inside your Linux Environment's base path: Install Sparkcan
1. sudo add-apt-repository ppa:graysonarendt/sparkcan
2. sudo apt update
3. sudo apt install sparkcan

### After Sparkcan Installation : Clone the Main Repo
1. mkdir robotics
2. cd robotics
3. git clone https://github.com/templerobotics/Lunabotics2024-ROS.git
4. cd Lunabotics2024-ROS
5. Type, "rm -rf build/ install/ log/" while inside of the Lunabotics2024-ROS directory to clear previous build artifacts 
6. chmod +x install_humble.sh --> The script will appear green in linux
7. ./install_humble.sh
8. In the Lunabotics2024-ROS directory type, "colcon build"


