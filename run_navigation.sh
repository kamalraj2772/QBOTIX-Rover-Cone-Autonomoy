#!/bin/bash
# ============================================================
# QBotix Rover - Autonomous Navigation Commands
# ============================================================
# Run these commands in separate terminals

echo "============================================================"
echo "QBotix Rover - Autonomous Navigation Setup"
echo "============================================================"

# ============================================================
# STEP 0: PREREQUISITES (Run once)
# ============================================================

echo "
=== STEP 0: Install micro-ROS Agent (Run once) ===

# Install micro-ROS agent
sudo apt update
sudo apt install -y ros-humble-micro-ros-agent

# OR build from source:
# mkdir -p ~/microros_ws/src
# cd ~/microros_ws/src
# git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
# cd ~/microros_ws
# colcon build
# source install/setup.bash
# ros2 run micro_ros_setup create_agent_ws.sh
# ros2 run micro_ros_setup build_agent.sh
"

# ============================================================
# STEP 1: TERMINAL 1 - micro-ROS Agent (for Teensy)
# ============================================================

echo "
=== TERMINAL 1: Start micro-ROS Agent ===

# Find Teensy port
ls /dev/ttyACM*

# Start micro-ROS agent (replace ttyACM0 if different)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
"

# ============================================================
# STEP 2: TERMINAL 2 - Main System Launch
# ============================================================

echo "
=== TERMINAL 2: Launch Full System ===

cd ~/ros2_ws
source install/setup.bash
ros2 launch qbotix_rover full_system.launch.py
"

# ============================================================
# STEP 3: TERMINAL 3 - Verification Commands
# ============================================================

echo "
=== TERMINAL 3: Verification Commands ===

# Source workspace first
source ~/ros2_ws/install/setup.bash

# --- Check Nodes ---
ros2 node list
# Expected: /zed/zed_node, /robot_state_publisher, /odom_publisher, 
#           /tf_broadcaster, /cone_detector, /controller_server, etc.

# --- Check Topics ---
ros2 topic list
# Expected: /cmd_vel, /odom, /tf, /cone_detections, /target_pose, etc.

# --- Check TF Tree ---
ros2 run tf2_ros tf2_echo map base_link
# Should show transform data

# --- View TF Tree Graph ---
ros2 run tf2_tools view_frames
# Creates frames.pdf

# --- Check Odometry Rate ---
ros2 topic hz /odom
# Should be ~30 Hz

# --- Check Motor RPM Feedback ---
ros2 topic echo /motor_rpm_feedback
# Shows [left_rpm, right_rpm] from Teensy

# --- Check cmd_vel ---
ros2 topic echo /cmd_vel
# Shows velocity commands from Nav2
"

# ============================================================
# STEP 4: TERMINAL 4 - Manual Motor Tests
# ============================================================

echo "
=== TERMINAL 4: Manual Motor Tests ===

source ~/ros2_ws/install/setup.bash

# --- Test Forward Motion (0.3 m/s) ---
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" --rate 10

# Press Ctrl+C to stop, then:

# --- Test Backward Motion ---
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" --rate 10

# --- Test Turn Left (in place) ---
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}\" --rate 10

# --- Test Turn Right (in place) ---
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}\" --rate 10

# --- Test Arc (forward + turn) ---
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}\" --rate 10

# --- STOP Motors ---
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\" --once
"

# ============================================================
# STEP 5: Send Navigation Goal (Autonomous)
# ============================================================

echo "
=== TERMINAL 5: Send Navigation Goals ===

source ~/ros2_ws/install/setup.bash

# --- Method 1: Use RViz ---
# Click '2D Goal Pose' button in RViz toolbar
# Click and drag on the map to set goal position and orientation

# --- Method 2: Command Line Goal (2m forward) ---
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \"
pose:
  header:
    frame_id: 'map'
  pose:
    position:
      x: 2.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
\"

# --- Method 3: Navigate to Detected Cone (automatic) ---
# The system automatically navigates to detected cones
# Check target pose:
ros2 topic echo /target_pose
"

# ============================================================
# STEP 6: Monitor Navigation
# ============================================================

echo "
=== TERMINAL 6: Monitor Navigation Status ===

source ~/ros2_ws/install/setup.bash

# --- Check Nav2 Status ---
ros2 topic echo /navigate_to_pose/_action/status

# --- Check Costmap ---
ros2 topic echo /local_costmap/costmap --once

# --- Check Controller Output ---
ros2 topic echo /cmd_vel

# --- Check Cone Detections ---
ros2 topic echo /cone_detections

# --- Monitor All Transforms ---
ros2 run tf2_ros tf2_monitor
"

# ============================================================
# TROUBLESHOOTING
# ============================================================

echo "
=== TROUBLESHOOTING ===

# --- If micro-ROS Agent fails to connect ---
# Check Teensy is connected:
ls -la /dev/ttyACM*
# Add user to dialout group:
sudo usermod -a -G dialout \$USER
# Logout and login again

# --- If TF frames missing ---
ros2 run tf2_tools view_frames
evince frames.pdf

# --- If Nav2 not starting ---
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
# If inactive, check for TF errors

# --- If motors not responding ---
# Check micro-ROS agent output for connection
# Check topic:
ros2 topic echo /cmd_vel
ros2 topic echo /motor_rpm_feedback

# --- Reset Nav2 ---
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes \"{command: 1}\"

# --- Emergency Stop ---
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.0}}\" --once
"

echo "============================================================"
echo "Commands saved. Open multiple terminals and run each step."
echo "============================================================"
