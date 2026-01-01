#!/bin/bash
# Automated verification script for QBotix Rover fixes

set -e

echo "=========================================="
echo "QBotix Rover - Verification Script"
echo "=========================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test counter
TESTS_PASSED=0
TESTS_FAILED=0

# Function to run test
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    echo -n "Testing: $test_name ... "
    if eval "$test_command" > /dev/null 2>&1; then
        echo -e "${GREEN}PASS${NC}"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}FAIL${NC}"
        ((TESTS_FAILED++))
        return 1
    fi
}

# Change to workspace
cd /home/qbotixrover/ros2_ws

echo "1. Checking Python Script Syntax"
echo "-----------------------------------"
run_test "cone_detector.py syntax" "python3 -m py_compile src/qbotix_perception/scripts/cone_detector.py"
run_test "target_publisher.py syntax" "python3 -m py_compile src/qbotix_perception/scripts/target_publisher.py"
run_test "depth_processor.py syntax" "python3 -m py_compile src/qbotix_perception/scripts/depth_processor.py"
echo ""

echo "2. Checking File Existence"
echo "-----------------------------------"
run_test "YOLO model exists" "test -f /home/qbotixrover/Documents/small.pt"
run_test "Launch file exists" "test -f src/qbotix_perception/launch/perception.launch.py"
run_test "Package installed" "test -d install/qbotix_perception"
echo ""

echo "3. Checking Package Build"
echo "-----------------------------------"
if run_test "Package builds" "colcon build --packages-select qbotix_perception --symlink-install 2>&1 | grep -q 'Finished'"; then
    echo "   ✓ Package built successfully"
fi
echo ""

echo "4. Checking Workspace Setup"
echo "-----------------------------------"
run_test "Setup script exists" "test -f install/setup.bash"
if source install/setup.bash 2>/dev/null; then
    echo -e "   ${GREEN}✓${NC} Workspace sourced successfully"
    ((TESTS_PASSED++))
else
    echo -e "   ${RED}✗${NC} Failed to source workspace"
    ((TESTS_FAILED++))
fi
echo ""

echo "5. Checking ROS2 Package"
echo "-----------------------------------"
source install/setup.bash
run_test "Package registered" "ros2 pkg list | grep -q qbotix_perception"
run_test "Executables installed" "test -f install/qbotix_perception/lib/qbotix_perception/cone_detector.py"
run_test "Launch files installed" "test -f install/qbotix_perception/share/qbotix_perception/launch/perception.launch.py"
echo ""

echo "6. Verifying Script Permissions"
echo "-----------------------------------"
run_test "cone_detector.py executable" "test -x src/qbotix_perception/scripts/cone_detector.py"
run_test "target_publisher.py executable" "test -x src/qbotix_perception/scripts/target_publisher.py"
run_test "depth_processor.py executable" "test -x src/qbotix_perception/scripts/depth_processor.py"
echo ""

echo "7. Checking Python Dependencies"
echo "-----------------------------------"
run_test "ultralytics (YOLO)" "python3 -c 'import ultralytics'"
run_test "opencv (cv2)" "python3 -c 'import cv2'"
run_test "numpy" "python3 -c 'import numpy'"
echo ""

echo "=========================================="
echo "Verification Summary"
echo "=========================================="
echo -e "Tests Passed: ${GREEN}${TESTS_PASSED}${NC}"
echo -e "Tests Failed: ${RED}${TESTS_FAILED}${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed! The system is ready.${NC}"
    echo ""
    echo "To launch the system, run:"
    echo "  ./test_cone_nav.sh"
    echo ""
    echo "Or run each component in separate terminals:"
    echo "  Terminal 1: ros2 launch qbotix_rover zed_camera.launch.py"
    echo "  Terminal 2: ros2 launch qbotix_rover rover_bringup.launch.py"
    echo "  Terminal 3: ros2 launch qbotix_navigation navigation.launch.py"
    echo "  Terminal 4: ros2 launch qbotix_perception perception.launch.py"
    exit 0
else
    echo -e "${RED}✗ Some tests failed. Please review the errors above.${NC}"
    echo ""
    echo "Common issues:"
    echo "  - Missing YOLO model: Download small.pt to /home/qbotixrover/Documents/"
    echo "  - Missing Python packages: pip install ultralytics opencv-python numpy"
    echo "  - Package not built: Run 'colcon build --packages-select qbotix_perception'"
    exit 1
fi
