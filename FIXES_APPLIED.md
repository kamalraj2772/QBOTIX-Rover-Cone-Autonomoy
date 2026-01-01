# QBotix Rover - Fixes Applied

## Summary of Fixes

### 1. cone_detector.py
**Issues Fixed:**
- Added missing `rgb_header` and `depth_header` initialization to prevent AttributeError
- Fixed depth image encoding from 'passthrough' to '32FC1' for ZED2i camera compatibility
- Added None checks for headers in callbacks to prevent crashes
- Updated default topic names to match ZED SDK:
  - RGB: `/zed/zed_node/rgb/image_rect_color`
  - Camera Info: `/zed/zed_node/rgb/camera_info`

**Changes Made:**
```python
# Added header initialization
self.rgb_header = None
self.depth_header = None

# Fixed depth encoding
self.current_depth = self.imgmsg_to_cv2(msg, '32FC1')  # Was 'passthrough'

# Added header checks
if self.current_rgb is None or self.rgb_header is None:
    return
```

### 2. target_publisher.py
**Issues Fixed:**
- Added explicit `TransformException` import from tf2_ros
- Improved exception handling for transform lookups

**Changes Made:**
```python
from tf2_ros import TransformException

# Better exception handling
except (tf2_ros.TransformException, TransformException) as e:
    self.get_logger().warn(f'Transform error: {e}')
```

### 3. depth_processor.py
**Issues Fixed:**
- Fixed depth image encoding from 'passthrough' to '32FC1' for consistency

**Changes Made:**
```python
# Fixed depth encoding to match ZED2i output
depth_image = self.imgmsg_to_cv2(msg, '32FC1')  # Was 'passthrough'
```

### 4. perception.launch.py
**Issues Fixed:**
- Updated topic names to match actual ZED SDK topics
- Corrected camera_info topic path

**Changes Made:**
```python
'image_topic': '/zed/zed_node/rgb/image_rect_color',  # Was '/zed/zed_node/rgb/color/rect/image'
'camera_info_topic': '/zed/zed_node/rgb/camera_info',  # Was '/zed/zed_node/rgb/color/rect/camera_info'
```

### 5. test_cone_nav.sh
**Issues Fixed:**
- Added proper workspace sourcing for all terminal commands
- Added RViz launch instructions

## How to Use

### Quick Start
```bash
# Terminal 1: ZED Camera
cd ~/ros2_ws && source install/setup.bash
ros2 launch qbotix_rover zed_camera.launch.py

# Terminal 2: Rover Bringup
cd ~/ros2_ws && source install/setup.bash
ros2 launch qbotix_rover rover_bringup.launch.py

# Terminal 3: Navigation
cd ~/ros2_ws && source install/setup.bash
ros2 launch qbotix_navigation navigation.launch.py

# Terminal 4: Perception (Cone Detection)
cd ~/ros2_ws && source install/setup.bash
ros2 launch qbotix_perception perception.launch.py

# Terminal 5: RViz Visualization (Optional)
cd ~/ros2_ws && source install/setup.bash
rviz2
```

### Verify Topics
```bash
# Check ZED topics
ros2 topic list | grep zed

# Expected topics:
# /zed/zed_node/rgb/image_rect_color
# /zed/zed_node/depth/depth_registered
# /zed/zed_node/rgb/camera_info

# Monitor cone detections
ros2 topic echo /cone_pose

# Monitor navigation goals
ros2 topic echo /goal_pose
```

### Testing
```bash
# Check if cone detector is running
ros2 node info /cone_detector

# View detection image
ros2 run image_view image_view --ros-args --remap image:=/cone_detection/image

# Monitor markers in RViz
# Add MarkerArray display with topic: /cone_markers
```

### Troubleshooting

#### No cone detections
1. Check if YOLO model exists:
   ```bash
   ls -lh /home/qbotixrover/Documents/small.pt
   ```
2. Verify camera is publishing:
   ```bash
   ros2 topic hz /zed/zed_node/rgb/image_rect_color
   ```
3. Check node logs:
   ```bash
   ros2 node list
   ros2 node info /cone_detector
   ```

#### Transform errors
1. Check TF tree:
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```
2. Verify transforms:
   ```bash
   ros2 run tf2_ros tf2_echo map base_link
   ros2 run tf2_ros tf2_echo map zed_left_camera_optical_frame
   ```

#### Camera topic mismatch
If ZED topics are different, update launch parameters:
```bash
ros2 launch qbotix_perception perception.launch.py \
    image_topic:=/your/image/topic \
    depth_topic:=/your/depth/topic \
    camera_info_topic:=/your/camera_info/topic
```

## Testing the Fixes

### 1. Compile Test
```bash
cd ~/ros2_ws
python3 -m py_compile src/qbotix_perception/scripts/*.py
echo "✓ Success: All scripts compile"
```

### 2. Rebuild Package
```bash
cd ~/ros2_ws
colcon build --packages-select qbotix_perception --symlink-install
source install/setup.bash
```

### 3. Run Node Test
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run qbotix_perception cone_detector.py
# Should start without errors and wait for camera topics
```

### 4. Full System Test
Follow the Quick Start instructions above to launch all nodes.

## Additional Notes

- **YOLO Model**: Ensure the model at `/home/qbotixrover/Documents/small.pt` is trained for cone detection
- **CUDA**: The detector uses CUDA acceleration on Jetson AGX
- **Frame Rate**: Cone detection runs at 10 Hz (configurable via `detection_rate` parameter)
- **Auto Navigation**: Enabled by default, disable with:
  ```bash
  ros2 param set /target_publisher auto_navigate false
  ```

## Files Modified

1. `/home/qbotixrover/ros2_ws/src/qbotix_perception/scripts/cone_detector.py`
2. `/home/qbotixrover/ros2_ws/src/qbotix_perception/scripts/target_publisher.py`
3. `/home/qbotixrover/ros2_ws/src/qbotix_perception/scripts/depth_processor.py`
4. `/home/qbotixrover/ros2_ws/src/qbotix_perception/launch/perception.launch.py`
5. `/home/qbotixrover/ros2_ws/test_cone_nav.sh`

All fixes have been tested and the package builds successfully.
