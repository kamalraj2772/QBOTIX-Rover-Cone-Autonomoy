#!/bin/bash
# QBotix Rover Setup Script
# For Jetson AGX with JetPack 6.1 and CUDA 12.6

set -e

echo "=========================================="
echo "QBotix Rover Setup Script"
echo "=========================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running on Jetson
if [ -f /etc/nv_tegra_release ]; then
    echo -e "${GREEN}Running on Jetson platform${NC}"
else
    echo -e "${YELLOW}Warning: Not running on Jetson platform${NC}"
fi

# Check ROS2 Humble installation
if [ -d "/opt/ros/humble" ]; then
    echo -e "${GREEN}ROS2 Humble found${NC}"
    source /opt/ros/humble/setup.bash
else
    echo -e "${RED}ROS2 Humble not found. Please install ROS2 Humble first.${NC}"
    echo "Installation guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html"
    exit 1
fi

# Install required system packages
echo ""
echo "Installing system dependencies..."
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    libopencv-dev \
    python3-opencv

# Install Python packages
echo ""
echo "Installing Python packages..."
pip3 install --upgrade pip
pip3 install \
    ultralytics \
    torch \
    torchvision \
    numpy \
    opencv-python

# Check for CUDA
if command -v nvcc &> /dev/null; then
    echo -e "${GREEN}CUDA found: $(nvcc --version | grep release)${NC}"
else
    echo -e "${YELLOW}Warning: CUDA not found in PATH${NC}"
fi

# Setup ZED SDK (if not installed)
echo ""
echo "Checking ZED SDK..."
if [ -d "/usr/local/zed" ]; then
    echo -e "${GREEN}ZED SDK found${NC}"
else
    echo -e "${YELLOW}ZED SDK not found. Please install from:${NC}"
    echo "https://www.stereolabs.com/developers/release"
    echo "For JetPack 6.1: Download ZED SDK for L4T 36.x"
fi

# Clone ZED ROS2 wrapper if not present
echo ""
echo "Setting up ZED ROS2 wrapper..."
ZED_WS="/home/qbotixrover/zed_ros2_ws"
if [ ! -d "$ZED_WS/src/zed-ros2-wrapper" ]; then
    mkdir -p $ZED_WS/src
    cd $ZED_WS/src
    git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
    cd $ZED_WS
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
    echo -e "${GREEN}ZED ROS2 wrapper built${NC}"
else
    echo -e "${GREEN}ZED ROS2 wrapper already present${NC}"
fi

# Build QBotix workspace
echo ""
echo "Building QBotix workspace..."
cd /home/qbotixrover/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release

echo ""
echo -e "${GREEN}=========================================="
echo "Setup complete!"
echo "==========================================${NC}"
echo ""
echo "To use the rover, add these to your .bashrc:"
echo ""
echo "  source /opt/ros/humble/setup.bash"
echo "  source /home/qbotixrover/zed_ros2_ws/install/setup.bash"
echo "  source /home/qbotixrover/ros2_ws/install/setup.bash"
echo ""
echo "Then run: ros2 launch qbotix_rover full_system.launch.py"
