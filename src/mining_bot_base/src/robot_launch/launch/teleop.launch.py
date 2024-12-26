# state_manager.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
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
   ])