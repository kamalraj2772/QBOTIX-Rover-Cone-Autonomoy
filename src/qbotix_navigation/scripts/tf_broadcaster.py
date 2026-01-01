#!/usr/bin/env python3
"""
TF Broadcaster for QBotix Rover
Manages all static and dynamic transforms
Subscribes to ZED odometry and publishes required TF frames
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class TFBroadcaster(Node):
    """
    Broadcasts required TF frames for the rover
    Subscribes to ZED odometry to publish odom->base_link
    """
    
    def __init__(self):
        super().__init__('tf_broadcaster')
        
        # Static TF broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Dynamic TF broadcaster
        self.dynamic_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Store latest odometry
        self.latest_odom = None
        self.odom_received = False
        
        # QoS for ZED topics (sensor data uses BEST_EFFORT)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to ZED odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/zed/zed_node/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Publish static transforms once
        self.publish_static_transforms()
        
        # Timer to publish transforms at 50Hz
        self.timer = self.create_timer(0.02, self.publish_transforms)
        
        self.get_logger().info('TF Broadcaster initialized')
        self.get_logger().info('Subscribing to /zed/zed_node/odom for odometry')
    
    def publish_static_transforms(self):
        """Publish all static transforms"""
        transforms = []
        
        # base_link -> zed_camera_link is handled by robot_state_publisher from URDF
        # ZED wrapper handles camera internal transforms
        
        if transforms:
            self.static_broadcaster.sendTransform(transforms)
            self.get_logger().info(f'Published {len(transforms)} static transforms')
    
    def odom_callback(self, msg: Odometry):
        """Store latest odometry from ZED"""
        self.latest_odom = msg
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('First odometry message received from ZED')
    
    def publish_transforms(self):
        """
        Publish all required TF transforms:
        - map -> odom (identity, no SLAM)
        - odom -> base_footprint (from ZED odometry)
        
        Note: base_footprint -> base_link is already defined in URDF (robot_state_publisher)
        """
        current_time = self.get_clock().now().to_msg()
        transforms = []
        
        # map -> odom (identity since no SLAM)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = current_time
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.x = 0.0
        t_map_odom.transform.rotation.y = 0.0
        t_map_odom.transform.rotation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0
        transforms.append(t_map_odom)
        
        # odom -> base_footprint (from ZED odometry or identity if no data yet)
        # Nav2 expects: map -> odom -> base_footprint -> base_link (from URDF)
        t_odom_base = TransformStamped()
        t_odom_base.header.stamp = current_time
        t_odom_base.header.frame_id = 'odom'
        t_odom_base.child_frame_id = 'base_footprint'
        
        if self.latest_odom is not None:
            # Use ZED odometry data
            t_odom_base.transform.translation.x = self.latest_odom.pose.pose.position.x
            t_odom_base.transform.translation.y = self.latest_odom.pose.pose.position.y
            t_odom_base.transform.translation.z = 0.0  # Keep footprint on ground plane
            t_odom_base.transform.rotation = self.latest_odom.pose.pose.orientation
        else:
            # Identity transform until we receive odometry
            t_odom_base.transform.translation.x = 0.0
            t_odom_base.transform.translation.y = 0.0
            t_odom_base.transform.translation.z = 0.0
            t_odom_base.transform.rotation.x = 0.0
            t_odom_base.transform.rotation.y = 0.0
            t_odom_base.transform.rotation.z = 0.0
            t_odom_base.transform.rotation.w = 1.0
        transforms.append(t_odom_base)
        
        # NOTE: base_footprint -> base_link is handled by robot_state_publisher from URDF
        # Do NOT publish it here to avoid TF tree loop!
        
        self.dynamic_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = TFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
