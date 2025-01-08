# Repo for Temple University ROS2-based Autonomous Mining Robot for NASA Lunabotics Competition

## Project Setup : Inside your Linux Environment's base path 
1. mkdir ~/robotics && cd robotics
2. git clone https://github.com/templerobotics/Lunabotics2024-ROS.git && cd Lunabotics2024-ROS
3. ./install_all_dependencies.sh
4. In the Lunabotics2024-ROS directory type, "build_hw_all" or look at "~/.bashrc" to see other possible build commands for the workspace.


## Documentation : Doxygen
In the " /robotics/Lunabotics2024-ROS/docs " folder lives doxygen documentation generation tool with folders html and latex

### To view Doxygen Documentation
1. cd to " /robotics/Lunabotics2024-ROS/docs/html " folder
2. Run " python3 -m http.server 8000 "  
3. Paste the port " http://localhost:8000/" into the browser




