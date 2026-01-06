import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("hexapod_bringup"),
        "config",
        "cmdvel_bridge.yaml",
    )

    teleop_config = os.path.join(
        get_package_share_directory("hexapod_bringup"),
        "config",
        "teleop_twist_joy.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy",
                output="screen",
                parameters=[teleop_config],
            ),
            Node(
                package="hexapod_cmdvel_bridge",
                executable="cmdvel_bridge",
                name="cmdvel_bridge",
                output="screen",
                parameters=[config],
            ),
        ]
    )
