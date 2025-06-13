from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    joy_node_jaden = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
    )

    drivebase = Node(
        package='robot',
        executable='drivebase',
        name='drivebase',
        output='screen'
    )

    input = Node(
        package='robot',
        executable='controller',
        name='controller',
        output='screen'
    )

    digging = Node(
        package='robot',
        executable='digging',
        name='digging',
        output='screen'
    )

    dumping = Node(
        package='robot',
        executable='dumping',
        name='dumping',
        output='screen'
    )

    autonomy = Node(
        package='robot',
        executable='autonomy',
        name='autonomy',
        output='screen'
    )

    idle = Node(
        package='robot',
        executable='idle',
        name='idle',
        output='screen'
    )


    joy_udp_listener_process_testing = ExecuteProcess(
        cmd=['python3', '/home/ubuntu/robotics/Lunabotics2024-ROS/src/upd_joy_listener/joy_udp_listener.py'],
        output='screen'
    )

    return LaunchDescription([
        joy_udp_listener_process_testing,
        drivebase,
        input,
        digging,
        dumping,
        autonomy,
        idle
    ])