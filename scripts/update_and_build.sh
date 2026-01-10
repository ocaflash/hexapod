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

# Clean build artifacts that may cause conflicts
cd "$WORKSPACE_DIR"
rm -rf build install log 2>/dev/null || true

# Rebuild: 1 package at a time, 2 compile threads per package
MAKEFLAGS="-j2" colcon build --symlink-install --parallel-workers 1 --cmake-clean-cache

echo "=== Update complete ==="
