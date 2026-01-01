#!/usr/bin/env python3
"""
YOLO Cone Detection Node for QBotix Rover
Detects cones using YOLOv8 model and publishes 3D positions
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import cv2
import math
import threading
from collections import deque

# Try to import cv_bridge, use fallback if NumPy incompatibility
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except (ImportError, AttributeError) as e:
    CV_BRIDGE_AVAILABLE = False
    print(f"WARNING: cv_bridge not available ({e}). Using fallback image conversion.")

# Import ultralytics for YOLOv8
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Install with: pip install ultralytics")


def imgmsg_to_cv2_fallback(msg, desired_encoding='bgr8'):
    """
    Fallback function to convert ROS Image message to OpenCV image
    when cv_bridge is not available due to NumPy incompatibility
    """
    dtype = np.uint8
    channels = 3
    
    # Determine dtype and channels based on encoding
    if msg.encoding == '32FC1':
        dtype = np.float32
        channels = 1
    elif msg.encoding == '16UC1':
        dtype = np.uint16
        channels = 1
    elif msg.encoding == 'mono8':
        dtype = np.uint8
        channels = 1
    elif msg.encoding in ['bgra8', 'rgba8']:
        dtype = np.uint8
        channels = 4
    elif msg.encoding in ['bgr8', 'rgb8']:
        dtype = np.uint8
        channels = 3
    
    img = np.frombuffer(msg.data, dtype=dtype)
    
    # Reshape based on channels
    if channels == 1:
        img = img.reshape((msg.height, msg.width))
    else:
        img = img.reshape((msg.height, msg.width, channels))
    
    # Handle color conversions
    if desired_encoding == 'bgr8':
        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'rgba8':
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        elif msg.encoding == 'bgra8':
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
    
    return img


def cv2_to_imgmsg_fallback(cv_image, encoding='bgr8'):
    """
    Fallback function to convert OpenCV image to ROS Image message
    when cv_bridge is not available
    """
    msg = Image()
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    
    # Calculate step (row stride in bytes)
    if len(cv_image.shape) == 2:
        # Grayscale image
        bytes_per_pixel = cv_image.dtype.itemsize
        msg.step = cv_image.shape[1] * bytes_per_pixel
    else:
        # Color image: width * channels * bytes_per_channel
        msg.step = cv_image.shape[1] * cv_image.shape[2] * cv_image.dtype.itemsize
    
    msg.data = cv_image.tobytes()
    return msg


class ConeDetector(Node):
    """
    ROS2 Node for detecting cones using YOLO and ZED2i camera
    """
    
    def __init__(self):
        super().__init__('cone_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', '/home/qbotixrover/Documents/medium.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/color/rect/image')  # Actual ZED topic
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/rgb/color/rect/camera_info')  # Actual camera info
        self.declare_parameter('detection_rate', 10.0)
        self.declare_parameter('cone_class_id', 0)  # Class ID for cone in your model
        self.declare_parameter('max_detection_distance', 10.0)
        self.declare_parameter('min_detection_distance', 0.3)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.detection_rate = self.get_parameter('detection_rate').value
        self.cone_class_id = self.get_parameter('cone_class_id').value
        self.max_detection_distance = self.get_parameter('max_detection_distance').value
        self.min_detection_distance = self.get_parameter('min_detection_distance').value
        
        # Initialize CV bridge if available
        self.bridge = None
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        
        # Initialize YOLO model
        self.model = None
        if YOLO_AVAILABLE:
            try:
                import os
                if not os.path.exists(self.model_path):
                    self.get_logger().error(f'YOLO model not found at: {self.model_path}')
                else:
                    self.model = YOLO(self.model_path)
                    # Try to use CUDA if available, fallback to CPU
                    try:
                        import torch
                        if torch.cuda.is_available():
                            self.model.to('cuda')
                            self.get_logger().info(f'YOLO model loaded from {self.model_path} (using CUDA)')
                        else:
                            self.get_logger().info(f'YOLO model loaded from {self.model_path} (using CPU)')
                    except:
                        self.get_logger().info(f'YOLO model loaded from {self.model_path} (using CPU)')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
        else:
            self.get_logger().error('YOLO not available. Please install ultralytics.')
        
        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        # Image buffers
        self.current_rgb = None
        self.current_depth = None
        self.rgb_header = None
        self.depth_header = None
        self.rgb_lock = threading.Lock()
        self.depth_lock = threading.Lock()
        
        # Detection history for smoothing
        self.detection_history = deque(maxlen=5)
        
        # QoS profile for sensor data - must match ZED publisher (RELIABLE)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.rgb_callback,
            sensor_qos
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            sensor_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.cone_pose_pub = self.create_publisher(
            PoseStamped,
            '/cone_pose',  # Direction arrow from robot to cone
            10
        )
        
        # Separate publisher for actual cone position (used by target_publisher)
        self.cone_position_pub = self.create_publisher(
            PoseStamped,
            '/cone_position',  # Actual cone location for navigation
            10
        )
        
        self.cone_markers_pub = self.create_publisher(
            MarkerArray,
            '/cone_markers',
            10
        )
        
        self.detection_image_pub = self.create_publisher(
            Image,
            '/cone_detection/image',
            10
        )
        
        self.target_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        # Detection timer
        self.timer = self.create_timer(1.0 / self.detection_rate, self.detection_callback)
        
        self.get_logger().info('Cone Detector Node initialized')
    
    def imgmsg_to_cv2(self, msg, encoding='bgr8'):
        """Convert ROS Image to CV2, using bridge if available or fallback"""
        if self.bridge is not None:
            return self.bridge.imgmsg_to_cv2(msg, encoding)
        else:
            return imgmsg_to_cv2_fallback(msg, encoding)
    
    def cv2_to_imgmsg(self, cv_image, encoding='bgr8'):
        """Convert CV2 to ROS Image, using bridge if available or fallback"""
        if self.bridge is not None:
            return self.bridge.cv2_to_imgmsg(cv_image, encoding)
        else:
            return cv2_to_imgmsg_fallback(cv_image, encoding)
    
    def rgb_callback(self, msg):
        """Callback for RGB image"""
        try:
            with self.rgb_lock:
                self.current_rgb = self.imgmsg_to_cv2(msg, 'bgr8')
                self.rgb_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Error converting RGB image: {e}')
    
    def depth_callback(self, msg):
        """Callback for depth image"""
        try:
            with self.depth_lock:
                # ZED2i publishes depth as 32FC1 (float32)
                self.current_depth = self.imgmsg_to_cv2(msg, '32FC1')
                self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def camera_info_callback(self, msg):
        """Callback for camera info"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info('Camera intrinsics received')
    
    def pixel_to_3d(self, u, v, depth):
        """
        Convert pixel coordinates and depth to 3D point in camera optical frame.
        Optical frame convention: X=right, Y=down, Z=forward (into the scene)
        """
        if self.fx is None:
            return None
        
        # In optical frame coordinates:
        # Z = depth (forward into scene)
        # X = horizontal offset (right positive)
        # Y = vertical offset (down positive)
        z = depth
        x = (u - self.cx) * z / self.fx  # Horizontal: right is positive
        y = (v - self.cy) * z / self.fy  # Vertical: down is positive
        
        # Return in optical frame convention: [X, Y, Z]
        return np.array([x, y, z])
    
    def get_depth_at_bbox(self, depth_image, bbox):
        """
        Get robust depth measurement within bounding box
        """
        x1, y1, x2, y2 = map(int, bbox)
        
        # Get center region of bounding box (to avoid edges)
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        w = (x2 - x1) // 4
        h = (y2 - y1) // 4
        
        roi = depth_image[max(0, cy-h):min(depth_image.shape[0], cy+h),
                          max(0, cx-w):min(depth_image.shape[1], cx+w)]
        
        # Check if ROI is valid
        if roi.size == 0:
            return None
        
        # Filter valid depth values
        valid_depths = roi[np.isfinite(roi) & (roi > 0) & (roi < self.max_detection_distance)]
        
        if len(valid_depths) > 0:
            # Use median for robustness
            return np.median(valid_depths)
        return None
    
    def detection_callback(self):
        """Main detection callback"""
        if self.model is None:
            return
        
        # Get current images
        with self.rgb_lock:
            if self.current_rgb is None or self.rgb_header is None:
                return
            rgb_image = self.current_rgb.copy()
            rgb_header = self.rgb_header
        
        with self.depth_lock:
            if self.current_depth is None or self.depth_header is None:
                return
            depth_image = self.current_depth.copy()
        
        if self.camera_matrix is None:
            return
        
        # Run YOLO detection
        try:
            results = self.model(rgb_image, verbose=False, conf=self.confidence_threshold)
        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')
            return
        
        # Process detections
        markers = MarkerArray()
        detection_image = rgb_image.copy()
        best_cone = None
        best_distance = float('inf')
        
        marker_id = 0
        
        for result in results:
            if result.boxes is None:
                continue
            
            for box in result.boxes:
                # Get class and confidence
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                
                # Check if it's a cone (adjust class ID as needed)
                # If using custom model, all detections might be cones
                bbox = box.xyxy[0].cpu().numpy()
                
                # Get depth at bounding box center
                depth = self.get_depth_at_bbox(depth_image, bbox)
                
                if depth is None or depth < self.min_detection_distance:
                    continue
                
                # Get 3D position
                cx = (bbox[0] + bbox[2]) / 2
                cy = (bbox[1] + bbox[3]) / 2
                
                point_3d = self.pixel_to_3d(cx, cy, depth)
                
                if point_3d is None:
                    continue
                
                # Draw on image
                x1, y1, x2, y2 = map(int, bbox)
                cv2.rectangle(detection_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f'Cone: {conf:.2f} D:{depth:.2f}m'
                cv2.putText(detection_image, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Create marker
                marker = Marker()
                marker.header.frame_id = 'zed_left_camera_optical_frame'  # Standard ROS frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'cones'
                marker.id = marker_id
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = point_3d[0]
                marker.pose.position.y = point_3d[1]
                marker.pose.position.z = point_3d[2]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.5
                marker.color.r = 1.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker.lifetime.sec = 1
                marker.lifetime.nanosec = 0
                markers.markers.append(marker)
                marker_id += 1
                
                # Track closest cone
                distance = np.linalg.norm(point_3d)
                if distance < best_distance:
                    best_distance = distance
                    best_cone = point_3d
        
        # Publish markers
        self.cone_markers_pub.publish(markers)
        
        # Publish detection image
        try:
            detection_msg = self.cv2_to_imgmsg(detection_image, 'bgr8')
            detection_msg.header = rgb_header
            self.detection_image_pub.publish(detection_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing detection image: {e}')
        
        # Publish best cone as target
        if best_cone is not None:
            # Publish cone pose AT THE CONE pointing BACK towards robot
            # Arrow shows "cone is here, robot should come this way"
            cone_pose = PoseStamped()
            cone_pose.header.frame_id = 'zed_left_camera_optical_frame'  # Standard ROS frame
            cone_pose.header.stamp = self.get_clock().now().to_msg()
            
            # Position AT the cone location
            cone_pose.pose.position.x = best_cone[0]
            cone_pose.pose.position.y = best_cone[1]
            cone_pose.pose.position.z = best_cone[2]
            
            # Orientation: arrow points FROM cone BACK TO robot (origin)
            # Direction vector from cone to origin is negative of cone position
            dx, dy, dz = -best_cone[0], -best_cone[1], -best_cone[2]
            
            # Calculate yaw (horizontal angle back to robot)
            yaw = math.atan2(dx, dz)    # Rotation around Y-axis
            
            # Quaternion for rotation around Y-axis
            cy = math.cos(yaw / 2)
            sy = math.sin(yaw / 2)
            cone_pose.pose.orientation.x = 0.0
            cone_pose.pose.orientation.y = sy  # Rotation around Y
            cone_pose.pose.orientation.z = 0.0
            cone_pose.pose.orientation.w = cy
            
            self.cone_pose_pub.publish(cone_pose)
            
            # Also publish the actual cone POSITION (for target_publisher)
            # This is separate from the direction arrow
            self.publish_cone_position(best_cone, best_distance)
    
    def publish_cone_position(self, cone_3d, distance):
        """Publish actual cone position for navigation (used by target_publisher)"""
        cone_pos = PoseStamped()
        cone_pos.header.frame_id = 'zed_left_camera_optical_frame'  # Standard ROS frame
        cone_pos.header.stamp = self.get_clock().now().to_msg()
        cone_pos.pose.position.x = cone_3d[0]
        cone_pos.pose.position.y = cone_3d[1]
        cone_pos.pose.position.z = cone_3d[2]
        cone_pos.pose.orientation.w = 1.0  # Identity orientation
        
        self.cone_position_pub.publish(cone_pos)
        self.get_logger().info(f'Cone at distance: {distance:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
