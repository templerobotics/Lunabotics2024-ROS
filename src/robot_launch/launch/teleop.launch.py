from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get directories
    robot_description_pkg = get_package_share_directory('robot_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    
    # Get file paths - updated for your package structure
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'david.urdf.xacro')
    controller_config_path = os.path.join(robot_description_pkg, 'config', 'gazebo_robot_params.yaml')
    gazebo_launch_path = os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={
            'pause': 'true',
            'verbose': 'true'
        }.items()
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mining_robot',
            '-topic', 'robot_description',
            '-z', '0.15'
        ],
        output='screen'
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )
    
    # Differential drive controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager'
        ],
        parameters=[controller_config_path],
        output='screen'
    )

    # Xbox controller node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )
    
    # Teleop state manager
    teleop_state_manager = Node(
        package='teleop_controller',
        executable='teleop_state_manager',
        name='teleop_state_manager',
        parameters=[{
            'XBOX': True,
            'PS4': False,
            'manual_enabled': True,
            'outdoor_mode': False,
            'robot_disabled': True
        }],
        output='screen'
    )
    
    # Teleop control node
    teleop_control = Node(
        package='teleop_controller',
        executable='teleop_control',
        name='teleop_control',
        output='screen'
    )

    return LaunchDescription([
        use_sim_time,
        # Launch Gazebo first
        gazebo,
        robot_state_publisher,
        
        # Spawn robot after Gazebo is ready
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo,
                on_exit=[spawn_robot]
            )
        ),
        
        # Launch controllers after robot is spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    joint_state_broadcaster_spawner,
                    diff_drive_controller_spawner
                ]
            )
        ),
        
        # Launch teleop nodes last
        joy_node,
        teleop_state_manager,
        teleop_control
    ])