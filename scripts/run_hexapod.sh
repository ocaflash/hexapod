#!/bin/bash
# Launch script for hexapod service

export HOME=/home/orangepi
export ROS_DOMAIN_ID=0

# Suppress FastDDS XML parser warnings (must be set before sourcing ROS)
export FASTRTPS_DEFAULT_PROFILES_FILE=""
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Clean ROS2 environment to avoid conflicts with other workspaces
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH

# Kill any existing hexapod processes
pkill -9 -f "ros2 launch hexapod" 2>/dev/null
pkill -9 -f "node_movement" 2>/dev/null
pkill -9 -f "node_servo" 2>/dev/null
pkill -9 -f "node_brain" 2>/dev/null
pkill -9 -f "cmdvel_bridge" 2>/dev/null
pkill -9 -f "joy_node" 2>/dev/null
pkill -9 -f "teleop_node" 2>/dev/null
sleep 0.5

# Release serial port if held
fuser -k /dev/ttyS5 2>/dev/null
sleep 0.2

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

# Initialize Mini Maestro UART communication
MAESTRO_PORT="/dev/ttyS5"
if [ -e "$MAESTRO_PORT" ]; then
    echo "Initializing Mini Maestro on $MAESTRO_PORT..."
    stty -F "$MAESTRO_PORT" 115200 cs8 -cstopb -parenb raw -echo -icanon min 0 time 5
    sleep 0.3
    # Send Go Home command (0xA2) to reset all servos
    printf '\xA2' > "$MAESTRO_PORT"
    sleep 0.2
    echo "Maestro initialized"
else
    echo "WARNING: Maestro port $MAESTRO_PORT not found"
fi

# Source ROS2 environment
source "$ROS_SETUP"
source "$WORKSPACE_DIR/install/local_setup.bash"

# Launch hexapod
exec ros2 launch hexapod_bringup target_launch.py
