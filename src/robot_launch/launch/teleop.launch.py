from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import subprocess

def generate_launch_description():
    # CAN interface setup commands. Need to mirrow how Sparkcan initializes CAN interface
    can_setup_cmd = ExecuteProcess(
        cmd=['bash', '-c', """
            sudo modprobe can
            sudo modprobe can_raw
            sudo modprobe peak_usb
            sudo ip link set can0 type can bitrate 1000000
            sudo ip link set up can0
        """],
        shell=True,
        output='screen'
    )

    hardware_enabled = DeclareLaunchArgument(
        'hardware_enabled',
        default_value='true',
        description='Enable hardware interfaces'
    )

    teleop_state_manager = Node(
        package='teleop_controller',
        executable='teleop_state_manager',
        name='teleop_state_manager',
        parameters=[{
            'XBOX': True,
            'manual_enabled': True,
            'outdoor_mode': False,
            'robot_disabled': True
        }],
        output='screen'
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
        can_setup_cmd,
        hardware_enabled,
        teleop_state_manager,
        drivebase_control
    ])