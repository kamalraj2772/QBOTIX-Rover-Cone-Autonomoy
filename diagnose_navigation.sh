#!/bin/bash
# Navigation Pipeline Diagnostic Script

echo "=========================================="
echo "Navigation Pipeline Diagnostics"
echo "=========================================="

source install/setup.bash

echo -e "\n1. CAMERA CHECK:"
echo "   RGB Camera:"
timeout 2 ros2 topic hz /zed/zed_node/rgb/image_rect_color 2>&1 | grep "average rate" || echo "   ❌ No RGB images"

echo "   Depth Camera:"
timeout 2 ros2 topic hz /zed/zed_node/depth/depth_registered 2>&1 | grep "average rate" || echo "   ❌ No depth images"

echo "   Camera Info:"
timeout 2 ros2 topic echo /zed/zed_node/rgb/camera_info --once >/dev/null 2>&1 && echo "   ✓ Camera info OK" || echo "   ❌ No camera info"

echo -e "\n2. CONE DETECTION CHECK:"
echo "   Detector node:"
ros2 node list | grep cone_detector >/dev/null && echo "   ✓ Cone detector running" || echo "   ❌ Cone detector NOT running"

echo "   Waiting 3 seconds for cone detection..."
if timeout 3 ros2 topic echo /cone_pose --once >/dev/null 2>&1; then
    echo "   ✓ CONE DETECTED!"
    ros2 topic echo /cone_pose --once 2>&1 | head -10
else
    echo "   ❌ NO CONE DETECTED"
    echo "   Troubleshooting:"
    echo "   - Is there a cone in camera view?"
    echo "   - Check detection image: ros2 run rqt_image_view rqt_image_view /cone_detection/image"
    echo "   - Check detector logs for errors"
fi

echo "   Detection rate:"
timeout 2 ros2 topic hz /cone_pose 2>&1 | grep "average rate" || echo "   No detections in 2 seconds"

echo -e "\n3. TARGET PUBLISHER CHECK:"
echo "   Node:"
ros2 node list | grep target_publisher >/dev/null && echo "   ✓ Target publisher running" || echo "   ❌ Target publisher NOT running"

echo "   Checking for cone in map frame:"
if timeout 3 ros2 topic echo /cone_pose_map --once >/dev/null 2>&1; then
    echo "   ✓ Cone transformed to map frame!"
    ros2 topic echo /cone_pose_map --once 2>&1 | head -10
else
    echo "   ❌ No cone in map frame (TF issue or no detection)"
fi

echo -e "\n4. GOAL GENERATION CHECK:"
echo "   Waiting for navigation goal..."
if timeout 3 ros2 topic echo /goal_pose --once >/dev/null 2>&1; then
    echo "   ✓ GOAL PUBLISHED!"
    ros2 topic echo /goal_pose --once 2>&1 | head -10
else
    echo "   ❌ NO GOAL being published"
    echo "   Check if auto_navigate is enabled:"
    ros2 param get /target_publisher auto_navigate 2>&1 || echo "   Can't read parameter"
fi

echo -e "\n5. PATH PLANNING CHECK:"
echo "   Planner status:"
ros2 lifecycle get /planner_server 2>&1 | grep -q "active" && echo "   ✓ Planner active" || echo "   ❌ Planner not active"

echo "   Checking for planned path..."
if timeout 3 ros2 topic echo /plan --once >/dev/null 2>&1; then
    echo "   ✓ PATH PLANNED!"
    POSES=$(timeout 1 ros2 topic echo /plan --once 2>&1 | grep -c "pose:")
    echo "   Path has $POSES poses"
else
    echo "   ❌ NO PATH being planned"
fi

echo -e "\n6. CONTROLLER CHECK:"
echo "   Controller status:"
ros2 lifecycle get /controller_server 2>&1 | grep -q "active" && echo "   ✓ Controller active" || echo "   ❌ Controller not active"

echo "   Checking cmd_vel output..."
if timeout 3 ros2 topic echo /cmd_vel --once >/dev/null 2>&1; then
    echo "   ✓ CMD_VEL BEING PUBLISHED!"
    ros2 topic echo /cmd_vel --once 2>&1 | head -6
    echo "   cmd_vel rate:"
    timeout 2 ros2 topic hz /cmd_vel 2>&1 | grep "average rate"
else
    echo "   ❌ NO CMD_VEL output"
fi

echo "   cmd_vel subscribers:"
SUBS=$(ros2 topic info /cmd_vel 2>&1 | grep "Subscription count:" | awk '{print $3}')
echo "   $SUBS subscriber(s)"

echo -e "\n7. TF TRANSFORMS CHECK:"
echo "   map -> base_link:"
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -3 && echo "   ✓ Transform OK" || echo "   ❌ Transform failed"

echo "   base_link -> camera:"
timeout 2 ros2 run tf2_ros tf2_echo base_link zed_left_camera_optical_frame 2>&1 | head -3 && echo "   ✓ Transform OK" || echo "   ❌ Transform failed"

echo -e "\n=========================================="
echo "DIAGNOSIS COMPLETE"
echo "=========================================="
echo ""
echo "COMMON FIXES:"
echo "1. No cone detected:"
echo "   - Place orange cone in front of camera"
echo "   - Check: ros2 run rqt_image_view rqt_image_view /cone_detection/image"
echo "   - Lower confidence: ros2 param set /cone_detector confidence_threshold 0.3"
echo ""
echo "2. No cmd_vel but cone detected:"
echo "   - Check goal manager: ros2 topic echo /goal_pose"
echo "   - Check navigation errors: ros2 topic echo /diagnostics"
echo ""
echo "3. Update rate issues:"
echo "   - Increase detection rate: ros2 param set /cone_detector detection_rate 15.0"
echo ""
echo "4. TF issues:"
echo "   - View TF tree: ros2 run tf2_tools view_frames"
