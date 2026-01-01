#!/bin/bash
# Quick test script for cone navigation
# Run each section in a SEPARATE terminal

echo "============================================================"
echo "QBotix Cone Navigation Test - Run in SEPARATE terminals!"
echo "============================================================"

echo "
=== TERMINAL 1: ZED Camera ===
cd ~/ros2_ws && source install/setup.bash
ros2 launch qbotix_rover zed_camera.launch.py

=== TERMINAL 2: Rover Bringup (robot model + TF) ===
cd ~/ros2_ws && source install/setup.bash
ros2 launch qbotix_rover rover_bringup.launch.py

=== TERMINAL 3: Navigation Stack ===
cd ~/ros2_ws && source install/setup.bash
ros2 launch qbotix_navigation navigation.launch.py

=== TERMINAL 4: Perception (cone detection) ===
cd ~/ros2_ws && source install/setup.bash
ros2 launch qbotix_perception perception.launch.py

=== TERMINAL 5: Monitor cmd_vel ===
cd ~/ros2_ws && source install/setup.bash
ros2 topic echo /cmd_vel

=== TERMINAL 6: Monitor cone detection ===
cd ~/ros2_ws && source install/setup.bash
ros2 topic echo /cone_pose

=== TO VIEW IN RVIZ: ===
cd ~/ros2_ws && source install/setup.bash
rviz2

=== TO SEND A TEST GOAL (place cone in front of robot): ===
# The system should automatically detect the cone and navigate to it.
# If auto-navigation is disabled, send a manual goal:
cd ~/ros2_ws && source install/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"
"
