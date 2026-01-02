#!/bin/bash
# Launch script for hexapod service

export HOME=/home/orangepi
export ROS_DOMAIN_ID=0

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/orangepi/hexapod_ws/install/local_setup.bash

# Launch hexapod
exec ros2 launch hexapod_bringup target_launch.py
