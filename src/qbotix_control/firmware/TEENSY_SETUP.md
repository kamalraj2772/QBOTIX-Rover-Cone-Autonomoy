# QBotix Rover - Teensy 4.1 micro-ROS Setup Guide

## Option 1: micro-ROS Arduino (Recommended)

### Step 1: Install Arduino IDE with Teensyduino
```bash
# Download Arduino IDE 1.8.x (NOT 2.x) from arduino.cc
# Download Teensyduino from pjrc.com/teensy/td_download.html
# Install Teensyduino on top of Arduino IDE
```

### Step 2: Install micro_ros_arduino Library
```bash
# In Arduino IDE:
# 1. Go to Sketch -> Include Library -> Manage Libraries
# 2. Search for "micro_ros_arduino"
# 3. Install the library

# OR manually:
cd ~/Arduino/libraries
git clone https://github.com/micro-ROS/micro_ros_arduino.git
```

### Step 3: Flash Teensy
1. Open Arduino IDE
2. Open: `~/ros2_ws/src/qbotix_control/firmware/teensy_microros_controller.ino`
3. Select: Tools -> Board -> Teensy 4.1
4. Select: Tools -> USB Type -> Serial
5. Click Upload

### Step 4: Install micro-ROS Agent on Jetson
```bash
# Install from apt
sudo apt update
sudo apt install -y ros-humble-micro-ros-agent

# Verify installation
ros2 pkg list | grep micro_ros
```

---

## Running the System

### Terminal 1: micro-ROS Agent
```bash
# Start agent to bridge Teensy to ROS2
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

Expected output:
```
[1234567890.123456] info     | TermiosAgentLinux.cpp | init                     | running...    | fd: 3
[1234567890.234567] info     | Root.cpp           | create_client            | create        | client_key: 0x12345678, session_id: 0x81
[1234567890.345678] info     | SessionManager.hpp | establish_session        | session established | client_key: 0x12345678, address: 0
```

### Terminal 2: Full Navigation System
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch qbotix_rover full_system.launch.py
```

### Terminal 3: Verify System
```bash
source ~/ros2_ws/install/setup.bash

# Check Teensy node is connected
ros2 node list | grep teensy
# Expected: /teensy_motor_controller

# Check topics from Teensy
ros2 topic list | grep motor
# Expected: /motor_rpm_feedback

# Check cmd_vel is being received
ros2 topic echo /cmd_vel

# Check motor feedback
ros2 topic echo /motor_rpm_feedback
```

---

## Testing Motors

### Test 1: Forward Motion
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10
```
- All wheels should spin forward
- LED on Teensy should be ON

### Test 2: Backward Motion
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3}, angular: {z: 0.0}}" --rate 10
```
- All wheels should spin backward

### Test 3: Turn Left (in place)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --rate 10
```
- Left wheels backward, right wheels forward

### Test 4: Turn Right (in place)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}" --rate 10
```
- Left wheels forward, right wheels backward

### Test 5: Stop
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

---

## Autonomous Navigation to Cone

### Method 1: Automatic (via YOLO detection)
The system automatically:
1. Detects cones with YOLO camera
2. Calculates 3D position from depth
3. Publishes to `/target_pose`
4. Nav2 navigates to target
5. MPPI avoids obstacles en route

Monitor with:
```bash
ros2 topic echo /cone_detections
ros2 topic echo /target_pose
```

### Method 2: Manual Goal via RViz
1. In RViz window, click "2D Goal Pose" button
2. Click on the visualization to set destination
3. Watch robot navigate autonomously

### Method 3: Command Line Goal
```bash
# Navigate to position (2m forward)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position: {x: 2.0, y: 0.0, z: 0.0}
    orientation: {w: 1.0}
"
```

---

## Verification Checklist

### ✅ Hardware Check
```bash
# Teensy connected
ls /dev/ttyACM0

# ZED camera connected  
ls /dev/video*
```

### ✅ Node Check
```bash
ros2 node list
# Should include:
# /teensy_motor_controller  (from micro-ROS)
# /zed/zed_node
# /robot_state_publisher
# /odom_publisher
# /tf_broadcaster
# /cone_detector
# /controller_server
# /planner_server
```

### ✅ TF Check
```bash
ros2 run tf2_ros tf2_echo map base_link
# Should output transform data

ros2 run tf2_tools view_frames
# Should create frames.pdf with complete tree
```

### ✅ Topic Check
```bash
# Odometry flowing
ros2 topic hz /odom
# Should be ~30 Hz

# Motor commands flowing
ros2 topic echo /cmd_vel

# Motor feedback from Teensy
ros2 topic echo /motor_rpm_feedback
```

### ✅ Navigation Check
```bash
# Nav2 lifecycle states
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
# Should be "active"
```

---

## LED Status on Teensy

| LED State | Meaning |
|-----------|---------|
| OFF | Connected to micro-ROS, no commands |
| ON | Receiving cmd_vel commands |
| Fast Blink | Error - not connected to agent |
| Slow Blink | Trying to connect |

---

## Troubleshooting

### Teensy not detected
```bash
# Check USB connection
dmesg | tail -20

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

### micro-ROS agent not connecting
```bash
# Kill any existing agents
pkill -f micro_ros_agent

# Restart with verbose output
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6
```

### Motors not responding
```bash
# Check if Teensy node exists
ros2 node list | grep teensy

# Check if cmd_vel is reaching Teensy
ros2 topic echo /cmd_vel

# Check Teensy is publishing feedback
ros2 topic echo /motor_rpm_feedback
```

### Nav2 not navigating
```bash
# Check if TF is complete
ros2 run tf2_ros tf2_echo map base_link

# Check lifecycle states
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server

# If inactive, restart:
ros2 lifecycle set /controller_server configure
ros2 lifecycle set /controller_server activate
```
