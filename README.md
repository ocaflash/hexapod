# Hexapod Robot (ROS2)

Open-source hexapod robot platform built with ROS2 for OrangePi Zero 2W.

## Hardware

- **SBC**: OrangePi Zero 2W with ROS2 Humble
- **Servo Controller**: Pololu Mini Maestro 18-Channel (UART)
- **Servos**: 18× MG996R (6 legs × 3 joints)
- **IMU**: MinIMU-9 v2 (I2C)
- **Controller**: Sony DualShock 4 v2 (Bluetooth)
- **Level Shifter**: BOB-08745 or HW-024 (for IMU 3.3V↔5V)

## Project Structure

```
hexapod_brain/           — High-level behavior and coordination
hexapod_movement/        — Gait controller and kinematics
hexapod_servo_controller/ — Pololu Maestro servo driver
hexapod_teleop/          — DualShock 4 teleoperation
hexapod_interfaces/      — ROS2 message definitions
hexapod_bringup/         — Launch files
```

## Quick Start

```bash
# Install dependencies
rosdep install --from-paths . --ignore-src -r -y

# Build
colcon build --symlink-install
source install/local_setup.bash

# Launch
ros2 launch hexapod_bringup target_launch.py
```

## Individual Components

```bash
ros2 launch hexapod_servo_controller servo_controller_launch.py
ros2 launch hexapod_movement movement_launch.py
ros2 launch hexapod_teleop teleop_launch.py
ros2 launch hexapod_brain brain_launch.py
```

## Wiring

```
OrangePi Zero 2W
    │
    ├── UART TX/RX ──► Mini Maestro 18-ch ──► 18× MG996R
    │                       │
    │                       └── External 6V PSU (10A min)
    │
    ├── I2C SDA/SCL ──► [Level Shifter] ──► MinIMU-9 v2
    │
    └── Bluetooth ──► DualShock 4
```

## Installation on OrangePi Zero 2W

```bash
# 1. Clone repository
mkdir -p ~/hexapod_ws/src
cd ~/hexapod_ws/src
git clone https://github.com/YOUR_USERNAME/hexapod_ros2.git

# 2. Run setup script
cd hexapod_ros2/scripts
chmod +x *.sh
./setup_workspace.sh

# 3. Install as service (optional, for autostart)
./install_service.sh
```

## Service Commands

```bash
sudo systemctl start hexapod     # Start robot
sudo systemctl stop hexapod      # Stop robot
sudo systemctl status hexapod    # Check status
sudo journalctl -u hexapod -f    # View logs
```

## Update from GitHub

```bash
~/hexapod_ws/src/hexapod_ros2/scripts/update_and_build.sh
```

## License

MIT License
