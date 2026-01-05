import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hexapod_teleop'),
        'config',
        'teleop.yaml'
    )
    return LaunchDescription([
        Node(
            package='hexapod_teleop',
            name='node_teleop',
            executable='node_teleop',
            output='screen',
            parameters=[config],
        )
    ])
