#!/bin/bash

echo "==========================================="
echo "Complete Navigation Pipeline Debugger"
echo "==========================================="
echo ""

cd ~/ros2_ws
source install/setup.bash

echo "Step 1: Check if cone is being detected..."
echo "   Waiting 3 seconds for cone detection..."
CONE_DETECTED=$(timeout 3 ros2 topic echo /cone_pose --once 2>&1)

if echo "$CONE_DETECTED" | grep -q "position:"; then
    echo "   ✓ CONE DETECTED!"
    echo "$CONE_DETECTED" | grep -A3 "position:"
    
    echo ""
    echo "Step 2: Check cone markers visualization..."
    MARKERS=$(timeout 1 ros2 topic echo /cone_markers --once 2>&1 | grep -c "marker")
    echo "   Markers published: $MARKERS"
    
    echo ""
    echo "Step 3: Check target_publisher node..."
    if ros2 node list | grep -q "target_publisher"; then
        echo "   ✓ target_publisher node running"
        
        echo ""
        echo "Step 4: Check TF transformation (cone → map)..."
        TF_CHECK=$(timeout 2 ros2 run tf2_ros tf2_echo zed_left_camera_optical_frame map 2>&1 | grep -c "Translation:")
        if [ "$TF_CHECK" -gt 0 ]; then
            echo "   ✓ TF transform exists"
        else
            echo "   ❌ TF transform FAILED"
            echo "   This is why goal_pose is not being published!"
        fi
        
        echo ""
        echo "Step 5: Check if goal is being published..."
        GOAL=$(timeout 2 ros2 topic echo /goal_pose --once 2>&1)
        if echo "$GOAL" | grep -q "position:"; then
            echo "   ✓ GOAL published!"
            echo "$GOAL" | grep -A3 "position:"
            
            echo ""
            echo "Step 6: Check nav2 planning..."
            PLAN=$(timeout 2 ros2 topic echo /plan --once 2>&1)
            if echo "$PLAN" | grep -q "poses:"; then
                echo "   ✓ PATH planned!"
                
                echo ""
                echo "Step 7: Check cmd_vel output..."
                CMD_VEL=$(timeout 2 ros2 topic echo /cmd_vel --once 2>&1)
                if echo "$CMD_VEL" | grep -q "linear:"; then
                    echo "   ✓ cmd_vel PUBLISHED!"
                    echo "$CMD_VEL"
                else
                    echo "   ❌ NO cmd_vel output"
                    echo "   Checking controller errors..."
                    timeout 2 ros2 topic echo /diagnostics --once 2>&1 | grep -A5 "controller"
                fi
            else
                echo "   ❌ NO PATH planned"
                echo "   Checking planner errors..."
                timeout 2 ros2 topic echo /diagnostics --once 2>&1 | grep -A5 "planner"
            fi
        else
            echo "   ❌ NO GOAL published"
            echo "   Checking target_publisher logs..."
            ros2 node info /target_publisher | grep -A10 "Publishers:"
        fi
    else
        echo "   ❌ target_publisher node NOT running!"
    fi
    
else
    echo "   ❌ NO CONE DETECTED"
    echo ""
    echo "Troubleshooting:"
    echo "   1. Place an ORANGE CONE in front of the camera"
    echo "   2. Check detection image:"
    echo "      ros2 run rqt_image_view rqt_image_view /cone_detection/image"
    echo "   3. Check camera is publishing:"
    echo "      ros2 topic hz /zed/zed_node/rgb/color/rect/image"
    echo "   4. Lower confidence threshold:"
    echo "      ros2 param set /cone_detector confidence_threshold 0.3"
fi

echo ""
echo "==========================================="
echo "QUICK COMMANDS:"
echo "==========================================="
echo "Check cone detection: ros2 topic hz /cone_pose"
echo "View detection image: rqt_image_view /cone_detection/image"
echo "Check cmd_vel output: ros2 topic echo /cmd_vel"
echo "Check goal generation: ros2 topic echo /goal_pose"
