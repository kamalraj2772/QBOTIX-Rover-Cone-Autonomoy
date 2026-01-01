#!/usr/bin/env python3
"""
Velocity Smoother for QBotix Rover
Smooths velocity commands to prevent jerky motion
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class VelocitySmoother(Node):
    """
    Smooths velocity commands using acceleration limits
    """
    
    def __init__(self):
        super().__init__('velocity_smoother')
        
        # Parameters
        self.declare_parameter('max_linear_accel', 1.0)  # m/s^2
        self.declare_parameter('max_angular_accel', 2.0)  # rad/s^2
        self.declare_parameter('max_linear_vel', 1.57)  # m/s
        self.declare_parameter('max_angular_vel', 1.57)  # rad/s
        self.declare_parameter('smoothing_rate', 50.0)  # Hz
        
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.smoothing_rate = self.get_parameter('smoothing_rate').value
        
        self.dt = 1.0 / self.smoothing_rate
        
        # Current and target velocities
        self.current_linear = 0.0
        self.current_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Subscriber for raw commands
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_callback,
            10
        )
        
        # Publisher for smoothed commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Smoothing timer
        self.timer = self.create_timer(self.dt, self.smooth_callback)
        
        self.get_logger().info('Velocity Smoother initialized')
    
    def cmd_callback(self, msg):
        """Store target velocities"""
        self.target_linear = max(-self.max_linear_vel, 
                                  min(self.max_linear_vel, msg.linear.x))
        self.target_angular = max(-self.max_angular_vel, 
                                   min(self.max_angular_vel, msg.angular.z))
    
    def smooth_callback(self):
        """Apply acceleration limits and publish smoothed velocity"""
        # Smooth linear velocity
        linear_diff = self.target_linear - self.current_linear
        max_linear_change = self.max_linear_accel * self.dt
        
        if abs(linear_diff) > max_linear_change:
            self.current_linear += math.copysign(max_linear_change, linear_diff)
        else:
            self.current_linear = self.target_linear
        
        # Smooth angular velocity
        angular_diff = self.target_angular - self.current_angular
        max_angular_change = self.max_angular_accel * self.dt
        
        if abs(angular_diff) > max_angular_change:
            self.current_angular += math.copysign(max_angular_change, angular_diff)
        else:
            self.current_angular = self.target_angular
        
        # Publish smoothed command
        msg = Twist()
        msg.linear.x = self.current_linear
        msg.angular.z = self.current_angular
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocitySmoother()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
