from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hexapod_teleop',
            name='node_teleop',
            executable='node_teleop',
            output='screen',
        )
    ])
