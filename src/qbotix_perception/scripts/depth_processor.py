#!/usr/bin/env python3
"""
Depth Processor Node
Processes ZED2i depth data for obstacle detection and costmap generation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import struct

# Try to import cv_bridge, use fallback if NumPy incompatibility
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except (ImportError, AttributeError) as e:
    CV_BRIDGE_AVAILABLE = False
    print(f"WARNING: cv_bridge not available ({e}). Using fallback image conversion.")


def imgmsg_to_cv2_fallback(msg, desired_encoding='passthrough'):
    """Fallback function to convert ROS Image message to OpenCV image"""
    dtype = np.uint8
    if msg.encoding == '32FC1':
        dtype = np.float32
    elif msg.encoding == '16UC1':
        dtype = np.uint16
    elif msg.encoding == 'mono8':
        dtype = np.uint8
    elif 'bgr8' in msg.encoding or 'rgb8' in msg.encoding:
        dtype = np.uint8
    
    img = np.frombuffer(msg.data, dtype=dtype)
    
    if msg.encoding == '32FC1' or msg.encoding == '16UC1' or msg.encoding == 'mono8':
        img = img.reshape((msg.height, msg.width))
    else:
        channels = 3 if '8' in msg.encoding else 4
        img = img.reshape((msg.height, msg.width, channels))
    
    return img


def cv2_to_imgmsg_fallback(cv_image, encoding='bgr8'):
    """Fallback function to convert OpenCV image to ROS Image message"""
    msg = Image()
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    
    if len(cv_image.shape) == 2:
        msg.step = cv_image.shape[1]
    else:
        msg.step = cv_image.shape[1] * cv_image.shape[2]
    
    msg.data = cv_image.tobytes()
    return msg


class DepthProcessor(Node):
    """
    Processes depth images and generates local obstacle information
    """
    
    def __init__(self):
        super().__init__('depth_processor')
        
        # Parameters
        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('min_obstacle_height', 0.15)  # meters
        self.declare_parameter('max_obstacle_height', 2.0)   # meters
        self.declare_parameter('obstacle_threshold', 0.5)    # meters
        self.declare_parameter('ground_clearance', 0.1)      # meters
        
        self.depth_topic = self.get_parameter('depth_topic').value
        self.min_obstacle_height = self.get_parameter('min_obstacle_height').value
        self.max_obstacle_height = self.get_parameter('max_obstacle_height').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.ground_clearance = self.get_parameter('ground_clearance').value
        
        # Initialize CV bridge if available
        self.bridge = None
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        
        self.camera_info = None
        
        # QoS for sensor data - ZED uses BEST_EFFORT for depth
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            sensor_qos
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/depth/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.obstacle_image_pub = self.create_publisher(
            Image,
            '/obstacle_detection/image',
            10
        )
        
        self.get_logger().info('Depth Processor Node initialized')
    
    def imgmsg_to_cv2(self, msg, encoding='passthrough'):
        """Convert ROS Image to CV2"""
        if self.bridge is not None:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
        else:
            return imgmsg_to_cv2_fallback(msg, encoding)
    
    def cv2_to_imgmsg(self, cv_image, encoding='bgr8'):
        """Convert CV2 to ROS Image"""
        if self.bridge is not None:
            return self.bridge.cv2_to_imgmsg(cv_image, encoding)
        else:
            return cv2_to_imgmsg_fallback(cv_image, encoding)
    
    def camera_info_callback(self, msg):
        """Store camera info"""
        self.camera_info = msg
    
    def depth_callback(self, msg):
        """Process depth image for obstacles"""
        try:
            # ZED2i publishes depth as 32FC1 (float32)
            depth_image = self.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
            return
        
        # Create obstacle visualization
        obstacle_vis = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), dtype=np.uint8)
        
        # Identify obstacles based on depth gradients
        valid_depth = np.where(np.isfinite(depth_image) & (depth_image > 0), depth_image, 0)
        
        # Calculate depth gradient
        grad_x = cv2.Sobel(valid_depth, cv2.CV_64F, 1, 0, ksize=5)
        grad_y = cv2.Sobel(valid_depth, cv2.CV_64F, 0, 1, ksize=5)
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # Threshold for obstacles
        obstacles = (gradient_magnitude > self.obstacle_threshold) & (valid_depth > 0)
        
        # Color coding
        # Green - free space
        # Red - obstacles
        # Yellow - close objects
        
        free_space = (valid_depth > 0) & ~obstacles
        close_objects = (valid_depth > 0) & (valid_depth < 1.5)
        
        obstacle_vis[free_space] = [0, 100, 0]  # Green
        obstacle_vis[obstacles] = [0, 0, 255]    # Red
        obstacle_vis[close_objects] = [0, 255, 255]  # Yellow
        
        # Publish visualization
        try:
            vis_msg = self.cv2_to_imgmsg(obstacle_vis, 'bgr8')
            vis_msg.header = msg.header
            self.obstacle_image_pub.publish(vis_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing obstacle image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
