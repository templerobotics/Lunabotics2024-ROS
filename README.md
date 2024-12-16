# Repo for Temple University ROS2-based Autonomous Mining Robot for NASA Lunabotics Competition

## How to Utilize this repo:

### Inside your Linux Environment's base path: Install Sparkcan
sudo add-apt-repository ppa:graysonarendt/sparkcan
sudo apt update
sudo apt install sparkcan

### After Sparkcan Installation : Clone the Main Repo
1. mkdir robotics
2. cd robotics
3. git clone https://github.com/templerobotics/Lunabotics2024-ROS.git
4. cd Lunabotics2024-ROS
5. chmod +x install_humble.sh
6. ./install_humble.sh
7. colcon build


