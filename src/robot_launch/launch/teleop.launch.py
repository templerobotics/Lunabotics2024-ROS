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

    # servo_control = ExecuteProcess(
    #     cmd=['python3', '/home/ubuntu/robotics/Lunabotics2024-ROS/src/arduino_nano/servo.py'],
    #     output='screen'
    # )

    # foxglove_bridge = Node(
    #     package='foxglove_bridge',
    #     executable='foxglove_bridge',
    #     name='foxglove_bridge',
    #     output='screen'
    # )

    # --- First Camera (namespace: cam_c960) ---
    camera_input_1 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='cam_c960',
        parameters=[
            {'video_device': '/dev/video2'},
            {'image_size': [640, 480]},
            {'time_per_frame': [1, 30]},
        ],
        output='screen'
    )

    # --- Second Camera (namespace: cam_c961) ---
    camera_input_2 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='cam_c961',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_size': [640, 480]},
            {'time_per_frame': [1, 20]},
        ],
        output='screen'
    )
    camera_input_3 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='cam_c962',
        parameters=[
            {'video_device': '/dev/video4'},
            {'image_size': [640, 480]},
            {'time_per_frame': [1, 15]},
        ],
        output='screen'
    )

    # --- First Camera Compressed Topic ---
    camera_compressed_1 = Node(
        package='image_transport',
        executable='republish',
        name='compressor_c960',
        arguments=[
            'raw', 'compressed',
            '--ros-args',
            '-r', 'in:=/cam_c960/image_raw',
            '-r', 'compressed:=/cam_c960/image_raw/compressed1'
        ],
        output='screen'
    )

    # --- Second Camera Compressed Topic ---
    camera_compressed_2 = Node(
        package='image_transport',
        executable='republish',
        name='compressor_c961',
        arguments=[
            'raw', 'compressed',
            '--ros-args',
            '-r', 'in:=/cam_c961/image_raw',
            '-r', 'compressed:=/cam_c961/image_raw/compressed2'
        ],
        output='screen'
    )
    camera_compressed_3 = Node(
        package='image_transport',
        executable='republish',
        name='compressor_c962',
        arguments=[
            'raw', 'compressed',
            '--ros-args',
            '-r', 'in:=/cam_c962/image_raw',
            '-r', 'compressed:=/cam_c962/image_raw/compressed3'
        ],
        output='screen'
    )

    # --- Static TF between map and camera (so RViz stops complaining) ---
    
    # Static TF from map to cam_c960/camera
    static_tf_cam_c960 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera'],
        output='screen'
    )

    # (Optional) If you want to do the same for cam_c961:
    static_tf_cam_c961 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera'],
        output='screen'
    )
     # (Optional) If you want to do the same for cam_c961:
    static_tf_cam_c962 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera'],
        output='screen'
    )

    return LaunchDescription([
        joy_udp_listener_process,
        drivebase_control,
        mode_control,
        joy_node_jaden,
        digging_control,
        dumping_control
        # foxglove_bridge,
        # camera_input_1,
        # camera_input_2,
        # camera_input_3,
        # camera_compressed_1,
        # camera_compressed_2,
        # camera_compressed_3,
        # static_tf_cam_c960,
        # static_tf_cam_c961,
        # static_tf_cam_c962
        # servo_control
    ])