from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import subprocess

def generate_launch_description():
   
    joy_node_jaden = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        )

    drivebase_control = Node(
        package='teleop_controller',
        executable='drivebase_control',
        name='drivebase_control',
        output='screen'
    )

    digging_control = Node(
         package='teleop_controller',
         executable='digging',
         name='digging',
         output='screen'
     )

    dumping_control = Node(
         package='teleop_controller',
         executable='dumping_conveyor_belt',
         name='dumping_conveyor_belt',
         output='screen'
     )
    

    mode_control = Node(
        package='teleop_controller',
        executable='robot_mode',
        name='activate_mode',
        output='screen'
    )

    joy_udp_listener_process = ExecuteProcess(
        cmd=['python3', '/home/ubuntu/robotics/Lunabotics2024-ROS/src/udp_joy_receiver/joy_udp_listener.py'],
        output='screen'
    )


    return LaunchDescription([
        joy_udp_listener_process,
        drivebase_control,
        mode_control,
        joy_node_jaden,
        digging_control,
        dumping_control
    ])