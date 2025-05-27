from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- First Camera (namespace: cam_c960) ---
    camera_input_1 = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='cam_c960',
        parameters=[
            {'video_device': '/dev/video2'},
            {'image_size': [640, 480]},
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

    return LaunchDescription([
        camera_input_1,
        camera_input_2,
        camera_compressed_1,
        camera_compressed_2,
        static_tf_cam_c960,
        static_tf_cam_c961
    ])
