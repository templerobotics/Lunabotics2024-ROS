from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import subprocess

def generate_launch_description():
    hardware_enabled = DeclareLaunchArgument(
        'hardware_enabled',
        default_value='true',
        description='Enable hardware interfaces'
    )

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
        parameters=[{
            'hardware_enabled': LaunchConfiguration('hardware_enabled')
        }],
        output='screen'
    )

    return LaunchDescription([
        hardware_enabled,
        drivebase_control,
        joy_node_jaden
    ])