# QBotix 6-Wheeled Autonomous Rover

## Overview
This project implements an autonomous navigation system for a 6-wheeled skid steering rover using:
- **ZED2i Camera** for visual odometry and mapping
- **YOLO** for cone detection
- **Nav2 with MPPI Controller** for obstacle avoidance and path planning
- **ROS2 Humble** on Jetson AGX with JetPack 6.1

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        QBotix Rover System                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │   ZED2i      │───▶│  Perception  │───▶│   Nav2       │       │
│  │   Camera     │    │  (YOLO)      │    │   (MPPI)     │       │
│  └──────────────┘    └──────────────┘    └──────────────┘       │
│         │                   │                   │                │
│         ▼                   ▼                   ▼                │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐       │
│  │  Visual      │    │   Target     │    │   Skid       │       │
│  │  Odometry    │    │   Publisher  │    │   Steer Ctrl │       │
│  └──────────────┘    └──────────────┘    └──────────────┘       │
│         │                                       │                │
│         └───────────────────┬───────────────────┘                │
│                             ▼                                    │
│                    ┌──────────────┐                              │
│                    │    Motors    │                              │
│                    │   (6 wheels) │                              │
│                    └──────────────┘                              │
└─────────────────────────────────────────────────────────────────┘
```

## TF Tree

```
map
 └── odom (from ZED visual odometry)
      └── base_link
           ├── base_footprint
           ├── zed_camera_link
           │    ├── zed_camera_center
           │    ├── zed_left_camera_frame
           │    │    └── zed_left_camera_optical_frame
           │    └── zed_right_camera_frame
           │         └── zed_right_camera_optical_frame
           ├── rocker_left
           │    ├── front_left_wheel
           │    ├── mid_left_wheel
           │    └── bogie_left
           │         └── rear_left_wheel
           └── rocker_right
                ├── front_right_wheel
                ├── mid_right_wheel
                └── bogie_right
                     └── rear_right_wheel
```

## Hardware Requirements
- Jetson AGX Orin/Xavier
- JetPack 6.1 with CUDA 12.6
- ZED2i Camera
- 6-Wheel Skid Steering Rover
- Motor driver (CAN/Serial/PWM)

## Software Requirements
- ROS2 Humble
- Nav2
- ZED ROS2 Wrapper
- Ultralytics (YOLOv8)
- OpenCV
- PyTorch (with CUDA)

## Installation

### 1. Prerequisites
```bash
# Install ROS2 Humble (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop

# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-mppi-controller
```

### 2. Install ZED SDK
Download and install ZED SDK for JetPack 6.1 from:
https://www.stereolabs.com/developers/release

```bash
# After downloading
chmod +x ZED_SDK_*.run
./ZED_SDK_*.run
```

### 3. Clone and Build ZED ROS2 Wrapper
```bash
mkdir -p ~/zed_ros2_ws/src
cd ~/zed_ros2_ws/src
git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git

cd ~/zed_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

### 4. Build QBotix Packages
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/zed_ros2_ws/install/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

### 5. Install Python Dependencies
```bash
pip3 install ultralytics torch torchvision numpy opencv-python
```

### 6. Setup Environment
Add to your `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
source ~/zed_ros2_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Quick Start - Full System
```bash
ros2 launch qbotix_rover full_system.launch.py
```

### Individual Components

#### 1. Robot Description Only
```bash
ros2 launch qbotix_rover rover_bringup.launch.py
```

#### 2. ZED Camera Only
```bash
ros2 launch qbotix_rover zed_camera.launch.py
```

#### 3. Navigation Only
```bash
ros2 launch qbotix_navigation navigation.launch.py
```

#### 4. Perception Only
```bash
ros2 launch qbotix_perception perception.launch.py model_path:=/home/qbotixrover/Documents/small.pt
```

#### 5. Control Only
```bash
ros2 launch qbotix_control control.launch.py
```

## Configuration

### Navigation Parameters
Edit `src/qbotix_navigation/config/nav2_params.yaml` to adjust:
- MPPI controller parameters
- Costmap settings
- Planner settings
- Speed limits

### Perception Parameters
Edit `src/qbotix_perception/config/perception_params.yaml` to adjust:
- YOLO model path
- Detection confidence
- Goal offset distance

### Control Parameters
Edit `src/qbotix_control/config/control_params.yaml` to adjust:
- Wheel dimensions
- Max RPM
- Motor interface settings

## Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/odom` | nav_msgs/Odometry | Robot odometry |
| `/cone_pose` | geometry_msgs/PoseStamped | Detected cone position |
| `/goal_pose` | geometry_msgs/PoseStamped | Navigation goal |
| `/wheel_rpm` | std_msgs/Int32MultiArray | Wheel RPM commands |
| `/cone_markers` | visualization_msgs/MarkerArray | Cone visualizations |
| `/zed/zed_node/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | 3D point cloud |

## MPPI Controller

The MPPI (Model Predictive Path Integral) controller is configured for skid steering with:
- **Batch size**: 2000 samples
- **Time steps**: 56
- **Max velocity**: 1.57 m/s (150 RPM)
- **Motion model**: DiffDrive (differential drive)

Key critics:
- **ObstaclesCritic**: Avoids obstacles detected by ZED
- **GoalCritic**: Drives toward the target
- **PathFollowCritic**: Follows the global path
- **PreferForwardCritic**: Prefers forward motion

## Troubleshooting

### Camera not detected
```bash
# Check ZED connection
ls /dev/video*

# Reset USB
sudo usbreset /dev/bus/usb/XXX/YYY
```

### Navigation not starting
```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Check topics
ros2 topic list
ros2 topic echo /odom
```

### YOLO model errors
```bash
# Verify model file
ls -la /home/qbotixrover/Documents/small.pt

# Test YOLO
python3 -c "from ultralytics import YOLO; m = YOLO('/home/qbotixrover/Documents/small.pt')"
```

## File Structure

```
ros2_ws/
├── src/
│   ├── qbotix_rover/           # Robot description and bringup
│   │   ├── config/
│   │   ├── launch/
│   │   ├── urdf/
│   │   └── rviz/
│   ├── qbotix_navigation/      # Navigation stack
│   │   ├── config/
│   │   ├── launch/
│   │   └── scripts/
│   ├── qbotix_perception/      # YOLO cone detection
│   │   ├── config/
│   │   ├── launch/
│   │   └── scripts/
│   └── qbotix_control/         # Motor control
│       ├── config/
│       ├── launch/
│       └── scripts/
└── setup_rover.sh              # Setup script
```

## License
Apache-2.0
