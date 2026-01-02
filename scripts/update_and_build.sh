#!/bin/bash
# Update hexapod project from GitHub and rebuild

set -e

WORKSPACE_DIR="$HOME/hexapod_ws"
SRC_DIR="$WORKSPACE_DIR/src/hexapod_ros2"

echo "=== Updating Hexapod Project ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Pull latest changes
cd "$SRC_DIR"
git fetch origin
git reset --hard origin/main

# Rebuild
cd "$WORKSPACE_DIR"
colcon build --symlink-install

echo "=== Update complete ==="
