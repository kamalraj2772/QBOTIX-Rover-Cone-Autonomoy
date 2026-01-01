#!/usr/bin/env python3
"""
Target Publisher Node - With Persistent Cone Registration
Transforms cone detections from camera frame to map frame and publishes navigation goals.

FEATURES:
- Registers cone position when first detected
- Continues navigation even when cone is temporarily hidden (occluded)
- Avoids dynamic obstacles while heading to registered cone position
- Updates registered position when cone becomes visible again
- Configurable timeout for cone registration expiry
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs
import numpy as np
import math
from collections import deque
import time


class TargetPublisher(Node):
    """
    Transforms cone pose from camera frame to map frame and publishes as navigation goal.
    Implements persistent cone registration for navigation through occlusions.
    """
    
    def __init__(self):
        super().__init__('target_publisher')
        
        # ============================================================
        # PARAMETERS
        # ============================================================
        self.declare_parameter('goal_offset_distance', 0.5)      # Stop this far from cone
        self.declare_parameter('min_goal_update_distance', 0.1)  # Min distance change to update
        self.declare_parameter('auto_navigate', True)
        self.declare_parameter('smooth_goal', True)
        self.declare_parameter('smoothing_window', 5)
        
        # Cone registration parameters
        self.declare_parameter('cone_registration_timeout', 30.0)  # Seconds before forgetting cone
        self.declare_parameter('cone_update_threshold', 0.3)       # Max distance to update registered position
        self.declare_parameter('goal_reached_threshold', 0.3)      # Distance to consider goal reached
        self.declare_parameter('goal_publish_rate', 2.0)           # Hz for publishing registered goal
        self.declare_parameter('goal_stability_threshold', 0.15)   # Min change to publish new goal (reduces fluctuation)
        self.declare_parameter('robot_moving_threshold', 0.05)     # Min robot movement to consider moving
        
        # Get parameters
        self.goal_offset = self.get_parameter('goal_offset_distance').value
        self.min_update_dist = self.get_parameter('min_goal_update_distance').value
        self.auto_navigate = self.get_parameter('auto_navigate').value
        self.smooth_goal = self.get_parameter('smooth_goal').value
        self.smoothing_window = self.get_parameter('smoothing_window').value
        self.cone_timeout = self.get_parameter('cone_registration_timeout').value
        self.cone_update_threshold = self.get_parameter('cone_update_threshold').value
        self.goal_reached_threshold = self.get_parameter('goal_reached_threshold').value
        self.goal_publish_rate = self.get_parameter('goal_publish_rate').value
        self.goal_stability_threshold = self.get_parameter('goal_stability_threshold').value
        self.robot_moving_threshold = self.get_parameter('robot_moving_threshold').value
        
        # ============================================================
        # CONE REGISTRATION STATE
        # ============================================================
        self.registered_cone = None          # (x, y) in map frame
        self.registered_goal = None          # (x, y, yaw) - goal position
        self.cone_first_seen_time = None     # When cone was first registered
        self.cone_last_seen_time = None      # Last time cone was visible
        self.cone_confidence = 0.0           # Confidence in cone position (0-1)
        self.cone_detection_count = 0        # Number of detections
        self.is_cone_visible = False         # Currently visible?
        
        # Navigation state
        self.navigation_active = False
        self.goal_reached = False
        
        # ============================================================
        # TF AND POSE
        # ============================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Goal history for smoothing
        self.goal_history = deque(maxlen=self.smoothing_window)
        self.last_published_goal = None
        
        # Current robot pose
        self.current_pose = None
        self.odom_received = False
        
        # ============================================================
        # QOS PROFILES
        # ============================================================
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ============================================================
        # SUBSCRIBERS
        # ============================================================
        self.cone_pose_sub = self.create_subscription(
            PoseStamped,
            '/cone_position',  # Use actual cone position (not direction arrow)
            self.cone_pose_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            reliable_qos
        )
        
        self.enable_sub = self.create_subscription(
            Bool,
            '/enable_auto_navigation',
            self.enable_callback,
            10
        )
        
        # Reset registration command
        self.reset_sub = self.create_subscription(
            Bool,
            '/reset_cone_registration',
            self.reset_callback,
            10
        )
        
        # ============================================================
        # PUBLISHERS
        # ============================================================
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.transformed_cone_pub = self.create_publisher(
            PoseStamped,
            '/cone_pose_map',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/cone_nav_status',
            10
        )
        
        # ============================================================
        # TIMERS
        # ============================================================
        # Timer to publish registered goal even when cone not visible
        self.goal_timer = self.create_timer(
            1.0 / self.goal_publish_rate,
            self.publish_registered_goal
        )
        
        # Timer for status updates
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Debug timer
        self.create_timer(5.0, self.check_odom_status)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Target Publisher Node initialized')
        self.get_logger().info('  - Cone registration timeout: {:.1f}s'.format(self.cone_timeout))
        self.get_logger().info('  - Goal offset distance: {:.2f}m'.format(self.goal_offset))
        self.get_logger().info('  - Persistent navigation: ENABLED')
        self.get_logger().info('='*50)
    
    # ================================================================
    # CALLBACKS
    # ================================================================
    
    def check_odom_status(self):
        """Debug: Check if odometry is being received"""
        if not self.odom_received:
            self.get_logger().warn('No odometry received yet! Check /odom topic and QoS.')
    
    def odom_callback(self, msg):
        """Store current robot pose"""
        if not self.odom_received:
            self.get_logger().info('First odometry received!')
            self.odom_received = True
        self.current_pose = msg.pose.pose
        
        # Check if goal reached
        if self.registered_goal is not None and self.current_pose is not None:
            dist_to_goal = math.sqrt(
                (self.current_pose.position.x - self.registered_goal[0])**2 +
                (self.current_pose.position.y - self.registered_goal[1])**2
            )
            if dist_to_goal < self.goal_reached_threshold:
                if not self.goal_reached:
                    self.get_logger().info('🎯 GOAL REACHED! Stopping navigation.')
                    self.goal_reached = True
                    self.navigation_active = False
    
    def enable_callback(self, msg):
        """Enable/disable auto navigation"""
        self.auto_navigate = msg.data
        self.get_logger().info(f'Auto navigation: {"enabled" if self.auto_navigate else "disabled"}')
    
    def reset_callback(self, msg):
        """Reset cone registration"""
        if msg.data:
            self.reset_cone_registration()
            self.get_logger().info('🔄 Cone registration reset by command')
    
    def reset_cone_registration(self):
        """Clear all cone registration data"""
        self.registered_cone = None
        self.registered_goal = None
        self.cone_first_seen_time = None
        self.cone_last_seen_time = None
        self.cone_confidence = 0.0
        self.cone_detection_count = 0
        self.is_cone_visible = False
        self.navigation_active = False
        self.goal_reached = False
        self.goal_history.clear()
    
    # ================================================================
    # CONE DETECTION CALLBACK
    # ================================================================
    
    def cone_pose_callback(self, msg):
        """
        Process cone detection and update registration
        """
        if not self.auto_navigate:
            return
        
        if self.current_pose is None:
            self.get_logger().warn('Cannot process cone: No odometry data', throttle_duration_sec=2.0)
            return
        
        try:
            # Transform cone to map frame
            source_frame = msg.header.frame_id
            
            if not self.tf_buffer.can_transform('map', source_frame, rclpy.time.Time(), 
                                                 timeout=rclpy.duration.Duration(seconds=0.1)):
                self.get_logger().warn(f'Cannot transform from {source_frame} to map', throttle_duration_sec=2.0)
                return
            
            transform = self.tf_buffer.lookup_transform(
                'map', source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(msg, transform)
            transformed_pose.header.frame_id = 'map'
            transformed_pose.header.stamp = self.get_clock().now().to_msg()
            self.transformed_cone_pub.publish(transformed_pose)
            
            # Cone position in map frame
            cone_x = transformed_pose.pose.position.x
            cone_y = transformed_pose.pose.position.y
            
            current_time = time.time()
            self.is_cone_visible = True
            self.cone_last_seen_time = current_time
            
            # ============================================================
            # CONE REGISTRATION LOGIC
            # ============================================================
            
            if self.registered_cone is None:
                # FIRST DETECTION - Register the cone
                self.register_cone(cone_x, cone_y)
                self.get_logger().info(f'📍 NEW CONE REGISTERED at ({cone_x:.2f}, {cone_y:.2f})')
            else:
                # Check if this is the same cone or a new one
                dist_to_registered = math.sqrt(
                    (cone_x - self.registered_cone[0])**2 +
                    (cone_y - self.registered_cone[1])**2
                )
                
                if dist_to_registered < self.cone_update_threshold:
                    # Same cone - update position with weighted average
                    self.update_cone_position(cone_x, cone_y)
                else:
                    # Different cone or cone moved significantly
                    self.get_logger().info(
                        f'⚠️ Cone position changed significantly ({dist_to_registered:.2f}m). Updating registration.'
                    )
                    self.register_cone(cone_x, cone_y)
            
            # Increment detection count and confidence
            self.cone_detection_count += 1
            self.cone_confidence = min(1.0, self.cone_detection_count / 10.0)
            
            # Calculate and publish goal
            self.calculate_and_publish_goal()
            
        except (tf2_ros.TransformException, TransformException) as e:
            self.get_logger().warn(f'Transform error: {e}', throttle_duration_sec=2.0)
    
    # ================================================================
    # CONE REGISTRATION FUNCTIONS
    # ================================================================
    
    def register_cone(self, cone_x, cone_y):
        """Register a new cone position"""
        self.registered_cone = (cone_x, cone_y)
        self.cone_first_seen_time = time.time()
        self.cone_last_seen_time = time.time()
        self.cone_detection_count = 1
        self.cone_confidence = 0.1
        self.goal_reached = False
        self.navigation_active = True
        self.goal_history.clear()
    
    def update_cone_position(self, cone_x, cone_y):
        """Update registered cone position with weighted average"""
        if self.registered_cone is None:
            self.register_cone(cone_x, cone_y)
            return
        
        # Weighted average: give more weight to existing position as confidence increases
        alpha = 0.3  # Weight for new detection
        new_x = (1 - alpha) * self.registered_cone[0] + alpha * cone_x
        new_y = (1 - alpha) * self.registered_cone[1] + alpha * cone_y
        
        self.registered_cone = (new_x, new_y)
    
    def is_cone_registration_valid(self):
        """Check if cone registration is still valid"""
        if self.registered_cone is None:
            return False
        
        if self.cone_last_seen_time is None:
            return False
        
        # Check timeout
        time_since_seen = time.time() - self.cone_last_seen_time
        if time_since_seen > self.cone_timeout:
            self.get_logger().warn(
                f'⏰ Cone registration expired ({time_since_seen:.1f}s since last seen)'
            )
            self.reset_cone_registration()
            return False
        
        return True
    
    # ================================================================
    # GOAL CALCULATION AND PUBLISHING
    # ================================================================
    
    def calculate_and_publish_goal(self):
        """Calculate goal position and publish"""
        if not self.is_cone_registration_valid():
            return
        
        if self.goal_reached:
            return
        
        cone_x, cone_y = self.registered_cone
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # Direction from cone to robot
        dx = robot_x - cone_x
        dy = robot_y - cone_y
        distance_to_cone = math.sqrt(dx*dx + dy*dy)
        
        if distance_to_cone < 0.1:
            self.get_logger().info('Already at cone position')
            return
        
        # Normalize direction
        dx /= distance_to_cone
        dy /= distance_to_cone
        
        # Goal is offset from cone towards robot
        goal_x = cone_x + dx * self.goal_offset
        goal_y = cone_y + dy * self.goal_offset
        
        # Calculate goal orientation (facing the cone)
        goal_yaw = math.atan2(cone_y - goal_y, cone_x - goal_x)
        
        # Add to history for smoothing
        self.goal_history.append((goal_x, goal_y, goal_yaw))
        
        # Smooth goal
        if self.smooth_goal and len(self.goal_history) >= 2:
            avg_x = np.mean([g[0] for g in self.goal_history])
            avg_y = np.mean([g[1] for g in self.goal_history])
            avg_yaw = np.mean([g[2] for g in self.goal_history])
            goal_x, goal_y, goal_yaw = avg_x, avg_y, avg_yaw
        
        # Check if goal changed significantly (reduces cmd_vel fluctuation)
        if self.last_published_goal is not None:
            last_x, last_y, _ = self.last_published_goal
            goal_change = math.sqrt((goal_x - last_x)**2 + (goal_y - last_y)**2)
            if goal_change < self.goal_stability_threshold:
                # Goal hasn't changed enough, skip publishing
                return
        
        # Update registered goal
        self.registered_goal = (goal_x, goal_y, goal_yaw)
        self.navigation_active = True
        
        # Publish goal
        self.publish_goal(goal_x, goal_y, goal_yaw, distance_to_cone, is_visible=True)
    
    def publish_registered_goal(self):
        """
        Timer callback: Publish registered goal even when cone is not visible.
        This enables navigation through temporary occlusions.
        """
        if not self.auto_navigate:
            return
        
        if not self.is_cone_registration_valid():
            return
        
        if self.registered_goal is None:
            return
        
        if self.goal_reached:
            return
        
        if self.current_pose is None:
            return
        
        # Check if cone is currently visible
        time_since_seen = time.time() - self.cone_last_seen_time if self.cone_last_seen_time else 999
        self.is_cone_visible = time_since_seen < 0.5  # Visible if seen in last 0.5s
        
        # If cone not visible, keep publishing registered goal
        if not self.is_cone_visible:
            goal_x, goal_y, goal_yaw = self.registered_goal
            
            # Calculate current distance
            robot_x = self.current_pose.position.x
            robot_y = self.current_pose.position.y
            cone_x, cone_y = self.registered_cone
            distance_to_cone = math.sqrt(
                (robot_x - cone_x)**2 + (robot_y - cone_y)**2
            )
            
            self.get_logger().info(
                f'👻 Cone HIDDEN for {time_since_seen:.1f}s - Using registered position ({cone_x:.2f}, {cone_y:.2f})',
                throttle_duration_sec=2.0
            )
            
            self.publish_goal(goal_x, goal_y, goal_yaw, distance_to_cone, is_visible=False)
    
    def publish_goal(self, goal_x, goal_y, goal_yaw, distance_to_cone, is_visible=True):
        """Publish goal pose message"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(goal_x)
        goal.pose.position.y = float(goal_y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = float(math.sin(goal_yaw / 2))
        goal.pose.orientation.w = float(math.cos(goal_yaw / 2))
        
        self.goal_pub.publish(goal)
        self.last_published_goal = (goal_x, goal_y, goal_yaw)
        
        visibility_icon = "👁️" if is_visible else "👻"
        self.get_logger().info(
            f'{visibility_icon} Goal: ({goal_x:.2f}, {goal_y:.2f}) | Dist to cone: {distance_to_cone:.2f}m | Conf: {self.cone_confidence:.0%}',
            throttle_duration_sec=1.0
        )
    
    def publish_status(self):
        """Publish navigation status"""
        status = String()
        
        if self.registered_cone is None:
            status.data = "SEARCHING - No cone registered"
        elif self.goal_reached:
            status.data = f"REACHED - Goal at ({self.registered_goal[0]:.2f}, {self.registered_goal[1]:.2f})"
        elif self.is_cone_visible:
            status.data = f"TRACKING - Cone visible at ({self.registered_cone[0]:.2f}, {self.registered_cone[1]:.2f})"
        else:
            time_hidden = time.time() - self.cone_last_seen_time if self.cone_last_seen_time else 0
            status.data = f"NAVIGATING - Cone hidden for {time_hidden:.1f}s, using registered position"
        
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
