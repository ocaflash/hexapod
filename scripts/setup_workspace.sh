#!/bin/bash
# Setup script for hexapod ROS2 workspace on OrangePi Zero 2W
# Run once after cloning the repository

set -e

WORKSPACE_DIR="$HOME/hexapod_ws"
SRC_DIR="$WORKSPACE_DIR/src"
REPO_URL="https://github.com/YOUR_USERNAME/hexapod_ros2.git"  # TODO: Update with your repo URL

echo "=== Setting up Hexapod ROS2 Workspace ==="

# Create workspace structure
mkdir -p "$SRC_DIR"

# Clone or update repository
if [ -d "$SRC_DIR/hexapod_ros2" ]; then
    echo "Repository exists, pulling latest..."
    cd "$SRC_DIR/hexapod_ros2"
    git pull
else
    echo "Cloning repository..."
    cd "$SRC_DIR"
    git clone "$REPO_URL" hexapod_ros2
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# Install dependencies
echo "Installing dependencies..."
cd "$WORKSPACE_DIR"
sudo apt update
sudo apt install -y \
    python3-pygame \
    nlohmann-json3-dev \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-std-srvs

rosdep install --from-paths src --ignore-src -r -y || true

# Build workspace
echo "Building workspace..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install

# Add to bashrc if not already present
if ! grep -q "hexapod_ws" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Hexapod ROS2 Workspace" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source $WORKSPACE_DIR/install/local_setup.bash" >> ~/.bashrc
fi

echo "=== Setup complete ==="
echo "Run: source ~/.bashrc"
echo "Then: ros2 launch hexapod_bringup target_launch.py"
