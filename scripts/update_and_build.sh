#!/bin/bash
# Update hexapod project from GitHub and rebuild

set -e

WORKSPACE_DIR="$HOME/hexapod_ws"
SRC_DIR="$WORKSPACE_DIR/src/hexapod"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== Updating Hexapod Project ==="

# Read saved ROS2 setup path
if [ -f "$SCRIPT_DIR/../.ros2_setup_path" ]; then
    ROS_SETUP=$(cat "$SCRIPT_DIR/../.ros2_setup_path")
    source "$ROS_SETUP"
else
    echo "ERROR: Run setup_workspace.sh first"
    exit 1
fi

# Pull latest changes
cd "$SRC_DIR"
git fetch origin
git reset --hard origin/main

# Rebuild
cd "$WORKSPACE_DIR"
colcon build --symlink-install

echo "=== Update complete ==="
