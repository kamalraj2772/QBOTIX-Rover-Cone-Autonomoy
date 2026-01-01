#!/bin/bash
# System Verification Script

echo "========================================="
echo "QBotix Rover System Verification"
echo "========================================="

# Check if workspace is sourced
echo -e "\n1. Checking workspace..."
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✓ Workspace sourced"
else
    echo "✗ Run this from ~/ros2_ws"
    exit 1
fi

# Check running nodes
echo -e "\n2. Checking nodes..."
EXPECTED_NODES=("zed_node" "cone_detector" "controller_server" "planner_server")
for node in "${EXPECTED_NODES[@]}"; do
    if ros2 node list | grep -q "$node"; then
        echo "✓ $node running"
    else
        echo "✗ $node NOT running"
    fi
done

# Check topic rates
echo -e "\n3. Checking topic frequencies..."
echo "Camera (should be ~30 Hz):"
timeout 3 ros2 topic hz /zed/zed_node/rgb/image_rect_color 2>/dev/null | grep "average rate" || echo "  No data"

echo "Odometry (should be ~30-60 Hz):"
timeout 3 ros2 topic hz /odom 2>/dev/null | grep "average rate" || echo "  No data"

echo "cmd_vel (check for subscribers):"
CMD_VEL_SUBS=$(ros2 topic info /cmd_vel | grep "Subscription count" | awk '{print $3}')
if [ "$CMD_VEL_SUBS" -gt 0 ]; then
    echo "✓ $CMD_VEL_SUBS subscriber(s) on /cmd_vel"
else
    echo "✗ NO subscribers on /cmd_vel - motor control not running!"
fi

# Check TF transforms
echo -e "\n4. Checking TF transforms..."
if timeout 2 ros2 run tf2_ros tf2_echo map base_link >/dev/null 2>&1; then
    echo "✓ map -> base_link transform OK"
else
    echo "✗ map -> base_link transform failed"
fi

if timeout 2 ros2 run tf2_ros tf2_echo base_link zed_left_camera_optical_frame >/dev/null 2>&1; then
    echo "✓ base_link -> camera transform OK"
else
    echo "✗ base_link -> camera transform failed"
fi

# Check cone detection
echo -e "\n5. Checking cone detection..."
if ros2 topic list | grep -q "/cone_pose"; then
    echo "✓ Cone detection topics available"
    echo "  Waiting 3s for cone detection..."
    if timeout 3 ros2 topic echo /cone_pose --once >/dev/null 2>&1; then
        echo "  ✓ Cone detected!"
    else
        echo "  ⚠ No cone detected (place cone in view)"
    fi
else
    echo "✗ Cone detection not running"
fi

# Check Navigation ready
echo -e "\n6. Checking Navigation..."
NAV_STATUS=$(ros2 lifecycle list /controller_server 2>/dev/null | grep "active" | wc -l)
if [ "$NAV_STATUS" -gt 0 ]; then
    echo "✓ Navigation stack active"
else
    echo "✗ Navigation stack not active"
fi

echo -e "\n========================================="
echo "Verification complete!"
echo "========================================="
