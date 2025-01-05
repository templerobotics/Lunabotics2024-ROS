from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_description_pkg = get_package_share_directory('robot_description')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    robot_launch_pkg = get_package_share_directory('robot_launch')
    
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'robot.urdf.xml')
    controller_config_path = os.path.join(robot_description_pkg, 'config', 'controller_config.yaml')
    gazebo_launch_path = os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }],
        output='screen'
    )
    
   
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'pause': 'true'}.items()
    )
    
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
        emulate_tty=True,
        output='screen'
    )
    
    teleop_control = Node(
        package='teleop_controller',
        executable='teleop_control',
        name='teleop_control',
        emulate_tty=True,
        output='screen'
    )
    
    # Joint state broadcaster for simm
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # Diff drive controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        parameters=[controller_config_path],
        output='screen'
    )

    return LaunchDescription([
        # Launch Gazebo first
        gazebo,
        # Launch robot description and state publisher
        robot_state_publisher,
        # Spawn robot in Gazebo
        spawn_robot,
        joy_node,
        # Launch controllers
        joint_state_broadcaster,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[diff_drive_controller]
            )
        ),
        
        teleop_state_manager,
        teleop_control
    ])