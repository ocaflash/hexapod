#!/bin/bash
# Setup script for hexapod ROS2 workspace on OrangePi Zero 2W

set -e

WORKSPACE_DIR="$HOME/hexapod_ws"
SRC_DIR="$WORKSPACE_DIR/src"

echo "=== Setting up Hexapod ROS2 Workspace ==="

# Find ROS2 installation
ROS_SETUP=""
for distro in humble jazzy iron rolling; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then
        ROS_SETUP="/opt/ros/$distro/setup.bash"
        echo "Found ROS2 $distro"
        break
    fi
done

# Check alternative locations
if [ -z "$ROS_SETUP" ] && [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    ROS_SETUP="$HOME/ros2_ws/install/setup.bash"
    echo "Found ROS2 in ~/ros2_ws"
fi

if [ -z "$ROS_SETUP" ] && [ -f "$HOME/ros2_humble/install/setup.bash" ]; then
    ROS_SETUP="$HOME/ros2_humble/install/setup.bash"
    echo "Found ROS2 in ~/ros2_humble"
fi

if [ -z "$ROS_SETUP" ]; then
    echo "ERROR: ROS2 not found!"
    echo "Please specify ROS2 setup.bash location:"
    echo "  export ROS_SETUP=/path/to/ros2/setup.bash"
    echo "  ./setup_workspace.sh"
    exit 1
fi

# Source ROS2
source "$ROS_SETUP"

# Save ROS_SETUP path for other scripts
echo "$ROS_SETUP" > "$SRC_DIR/hexapod/.ros2_setup_path"

# Install dependencies
echo "Installing dependencies..."
cd "$WORKSPACE_DIR"
sudo apt update
sudo apt install -y \
    python3-pygame \
    nlohmann-json3-dev || true

rosdep install --from-paths src --ignore-src -r -y || true

# Build workspace (single job to reduce CPU/memory load on OrangePi)
echo "Building workspace..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install --parallel-workers 1 --executor sequential

# Add to bashrc if not already present
if ! grep -q "hexapod_ws" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Hexapod ROS2 Workspace" >> ~/.bashrc
    echo "source $ROS_SETUP" >> ~/.bashrc
    echo "source $WORKSPACE_DIR/install/local_setup.bash" >> ~/.bashrc
fi

echo "=== Setup complete ==="
echo "ROS2 setup: $ROS_SETUP"
echo ""
echo "Run: source ~/.bashrc"
echo "Then: ros2 launch hexapod_bringup target_launch.py"
