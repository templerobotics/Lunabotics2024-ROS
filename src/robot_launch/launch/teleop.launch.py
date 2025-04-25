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

    # V4L2 Camera Node (publishes /image_raw)
    camera_input = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        parameters=[{'image_size': [640, 480], 'time_per_frame': [1, 10]}],  # ~10 FPS
        remappings=[
            ('/image_raw', '/camera/image_raw')
        ],
        output='screen'
    )

    # Image Republisher Node (compresses and republishes)
    camera_compressed = Node(
        package='image_transport',
        executable='republish',
        name='image_transport_republish',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out', '/camera/image_raw/compressed')
        ],
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
        camera_input,
        camera_compressed
    ])
