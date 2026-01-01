#!/usr/bin/env python3
"""
Odometry Publisher for QBotix Rover
Uses ZED2i visual odometry for robot localization
Republishes ZED odometry on standard /odom topic and publishes odom->base_link TF
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, PoseWithCovarianceStamped
import tf2_ros
import math
import numpy as np


class OdomPublisher(Node):
    """
    Republishes ZED2i visual odometry on /odom topic.
    Also publishes odom->base_link TF since ZED might use different frame names.
    """
    
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')  # Nav2 expects base_footprint
        self.declare_parameter('zed_odom_topic', '/zed/zed_node/odom')
        self.declare_parameter('publish_tf', False)  # TF is published by tf_broadcaster
        
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # TF broadcaster for odom->base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # QoS for ZED topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber for ZED odometry
        self.zed_odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('zed_odom_topic').value,
            self.zed_odom_callback,
            sensor_qos
        )
        
        # Publisher for robot odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        self.last_odom_time = None
        self.odom_count = 0
        
        # Timer for publishing rate monitoring
        self.rate_timer = self.create_timer(5.0, self.log_rate)
        
        self.get_logger().info('Odometry Publisher initialized')
        self.get_logger().info(f'Republishing ZED odometry to /odom with TF: {self.publish_tf}')
    
    def log_rate(self):
        """Log odometry rate for debugging"""
        if self.odom_count > 0:
            self.get_logger().debug(f'Odom rate: {self.odom_count / 5.0:.1f} Hz')
        self.odom_count = 0
    
    def zed_odom_callback(self, msg):
        """
        Handle ZED odometry, republish on /odom, and publish TF
        """
        # Create odometry message with standard frame names
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Copy pose from ZED
        odom.pose = msg.pose
        
        # Copy twist
        odom.twist = msg.twist
        
        # Ensure covariance is set if not present
        if all(c == 0.0 for c in odom.pose.covariance):
            # Set reasonable default covariance for visual odometry
            odom.pose.covariance[0] = 0.01   # x
            odom.pose.covariance[7] = 0.01   # y
            odom.pose.covariance[14] = 0.01  # z
            odom.pose.covariance[21] = 0.01  # roll
            odom.pose.covariance[28] = 0.01  # pitch
            odom.pose.covariance[35] = 0.01  # yaw
        
        # Publish odometry
        self.odom_pub.publish(odom)
        self.odom_count += 1
        
        # Publish TF odom -> base_link
        if self.publish_tf:
            self.publish_odom_tf(msg)
    
    def publish_odom_tf(self, msg):
        """Publish the odom->base_link transform from ZED odometry"""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        # Extract pose from odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
