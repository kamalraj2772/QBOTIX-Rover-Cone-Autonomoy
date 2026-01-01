# QBotix Rover - Quick Start Guide

## System Overview
- **Platform**: Jetson AGX with JetPack 6.1, CUDA 12.6
- **Camera**: ZED2i for visual odometry and depth sensing
- **Controller**: Teensy 4.1 with MDS40B Cytron motor drivers
- **Navigation**: ROS2 Humble with Nav2 MPPI controller
- **Detection**: YOLOv8 for cone detection

## Hardware Connections

### Teensy 4.1 Motor Pins (MDS40B Anti-Locked Phase)
```
Right Side Motors:
  R1_PWM -> Pin 18 (Front Right)
  R2_PWM -> Pin 23 (Middle Right)
  R3_PWM -> Pin 16 (Rear Right)

Left Side Motors:
  L1_PWM -> Pin 12 (Front Left)
  L2_PWM -> Pin 27 (Middle Left)
  L3_PWM -> Pin 26 (Rear Left)
```

### MDS40B Wiring (Anti-Locked Phase Mode)
- Each motor driver receives single PWM signal
- PWM > 50% = Forward rotation
- PWM < 50% = Reverse rotation
- PWM = 50% = Stop

---

## Step 1: Flash Teensy Firmware

1. Open Arduino IDE with Teensyduino installed
2. Open: `src/qbotix_control/firmware/teensy_motor_controller.ino`
3. Select Board: "Teensy 4.1"
4. Upload to Teensy

---

## Step 2: Connect Hardware

1. Connect ZED2i camera via USB 3.0
2. Connect Teensy 4.1 via USB (will appear as /dev/ttyACM0)
3. Verify connections:
```bash
# Check ZED camera
ls /dev/video*

# Check Teensy
ls /dev/ttyACM*
```

---

## Step 3: Launch the System

### Terminal 1: Full System Launch
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch qbotix_rover full_system.launch.py
```

This launches:
- Robot state publisher (URDF)
- ZED2i camera (visual odometry, depth, point cloud)
- TF broadcaster (map -> odom)
- Odometry publisher (odom -> base_link)
- Nav2 stack with MPPI controller
- Skid steer controller
- Motor interface (Teensy communication)
- Cone detector (YOLO)
- Target publisher
- RViz for visualization

---

## Step 4: Verify System

### Check TF Tree
```bash
# In new terminal
source ~/ros2_ws/install/setup.bash
ros2 run tf2_tools view_frames
```

Expected frames: `map -> odom -> base_link -> zed_camera_link -> zed_left_camera_frame`

### Check Topics
```bash
ros2 topic list | grep -E "cmd_vel|odom|motor|cone"
```

Expected topics:
- `/cmd_vel` - Velocity commands from Nav2
- `/odom` - Odometry from ZED
- `/motor_rpm` - RPM commands to motors
- `/motor_feedback` - Feedback from Teensy
- `/cone_detections` - Detected cones
- `/target_pose` - Target pose for navigation

### Check Motor Communication
```bash
# Monitor motor commands
ros2 topic echo /motor_rpm

# Monitor Teensy feedback
ros2 topic echo /motor_feedback
```

---

## Step 5: Test Motors

### Manual Motor Test
```bash
# Publish test velocity (0.5 m/s forward)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -1
```

---

## Step 6: Autonomous Navigation to Cone

The system automatically:
1. Detects cones using YOLO (`/cone_detections`)
2. Calculates 3D position using depth (`/target_pose`)
3. Sends navigation goal to Nav2
4. MPPI controller plans path avoiding obstacles
5. Commands sent to motors via Teensy

### Monitor Cone Detection
```bash
ros2 topic echo /cone_detections
```

### Set Manual Navigation Goal (RViz)
- Click "2D Goal Pose" in RViz
- Click on map to set destination

---

## Troubleshooting

### No Motor Response
```bash
# Check Teensy connection
dmesg | tail -20

# Check if Teensy is detected
ls -la /dev/ttyACM*

# Test direct serial communication
screen /dev/ttyACM0 115200
# Type: 50.0,50.0
# Should respond: OK L:50.0 R:50.0
```

### No TF Frames
```bash
# Check if odom_publisher is running
ros2 node list | grep odom

# Check TF
ros2 run tf2_ros tf2_echo map base_link
```

### Nav2 Not Starting
```bash
# Check Nav2 lifecycle
ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"

# Check controller server
ros2 lifecycle get /controller_server
```

### ZED Camera Issues
```bash
# Check ZED node
ros2 node info /zed/zed_node

# Check if odometry is publishing
ros2 topic hz /zed/zed_node/odom
```

---

## ROS2 Topic Map

```
/cmd_vel (Twist)
    └──> skid_steer_controller
         └──> /motor_rpm (Float64MultiArray[left, right])
              └──> motor_interface
                   └──> Teensy Serial ("left,right\n")
                        └──> MDS40B PWM signals

/zed/zed_node/odom (Odometry)
    └──> odom_publisher
         └──> /odom (Odometry)
         └──> TF: odom -> base_link

/zed/zed_node/rgb/image_rect_color (Image)
    └──> cone_detector (YOLO)
         └──> /cone_detections
              └──> target_publisher
                   └──> /target_pose (PoseStamped)
                        └──> goal_manager
                             └──> Nav2 navigate_to_pose
```

---

## Key Parameters

### Speed Limits (nav2_params.yaml)
- `vx_max`: 1.57 m/s (from 150 RPM)
- `wz_max`: 1.57 rad/s

### Robot Dimensions
- Wheel radius: 0.10 m
- Wheel separation: 0.53 m
- Max RPM: 150

### MPPI Controller
- Lookahead: 3.0 m
- Time steps: 56
- Model: DiffDrive (skid steering approximation)
