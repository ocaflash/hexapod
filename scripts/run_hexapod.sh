#!/bin/bash
# Launch script for hexapod service

export HOME=/home/orangepi
export ROS_DOMAIN_ID=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$HOME/hexapod_ws"

# Read saved ROS2 setup path
if [ -f "$SCRIPT_DIR/../.ros2_setup_path" ]; then
    ROS_SETUP=$(cat "$SCRIPT_DIR/../.ros2_setup_path")
else
    # Fallback: try to find ROS2
    for distro in humble jazzy iron rolling; do
        if [ -f "/opt/ros/$distro/setup.bash" ]; then
            ROS_SETUP="/opt/ros/$distro/setup.bash"
            break
        fi
    done
    [ -z "$ROS_SETUP" ] && [ -f "$HOME/ros2_ws/install/setup.bash" ] && ROS_SETUP="$HOME/ros2_ws/install/setup.bash"
    [ -z "$ROS_SETUP" ] && [ -f "$HOME/ros2_humble/install/setup.bash" ] && ROS_SETUP="$HOME/ros2_humble/install/setup.bash"
fi

if [ -z "$ROS_SETUP" ]; then
    echo "ERROR: ROS2 not found!"
    exit 1
fi

# Source ROS2 environment
source "$ROS_SETUP"
source "$WORKSPACE_DIR/install/local_setup.bash"

# Launch hexapod
exec ros2 launch hexapod_bringup target_launch.py
