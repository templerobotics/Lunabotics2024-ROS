from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    mac_ip = '192.168.1.112'

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

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    camera_input_1 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_c960',
        parameters=[
            {'video_device': '/dev/video0'},
            {'camera_name': 'cam_c960'},
            {'image_size': [640, 480]},  # Smaller resolution if you want
        ],
        output='screen'
    )

    # Second Camera
    camera_input_2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_c961',
        parameters=[
            {'video_device': '/dev/cam_c961'},
            {'camera_name': 'cam_c961'},
            {'image_size': [640, 480]},  # Smaller resolution if you want
        ],
        output='screen'
    )

    camera_compressed_1 = Node(
        package='image_transport',
        executable='republish',
        name='compressor_c960',
        arguments=['raw', 'compressed', '--ros-args', '-r', 'in:=/cam_c960/image_raw', '-r', 'out:=/cam_c960/image_raw/compressed'],
        output='screen'
    )
    camera_compressed_2 = Node(
        package='image_transport',
        executable='republish',
        name='compressor_c961',
        arguments=['raw', 'compressed', '--ros-args', '-r', 'in:=/cam_c961/image_raw', '-r', 'out:=/cam_c961/image_raw/compressed'],
        output='screen'
    )

    return LaunchDescription([
        joy_udp_listener_process,
        drivebase_control,
        mode_control,
        joy_node_jaden,
        digging_control,
        dumping_control,
        foxglove_bridge,
        camera_input_1,
        camera_input_2,
        camera_compressed_1,
        camera_compressed_2
    ])