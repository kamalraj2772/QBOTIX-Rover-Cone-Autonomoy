#!/usr/bin/env python3
"""
Skid Steer Controller for QBotix 6-Wheeled Rover
Converts cmd_vel to individual wheel velocities for skid steering
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import math


class SkidSteerController(Node):
    """
    Controller for 6-wheeled skid steering rover
    Converts Twist commands to wheel velocities
    """
    
    def __init__(self):
        super().__init__('skid_steer_controller')
        
        # Robot parameters
        self.declare_parameter('wheel_radius', 0.10)  # meters
        self.declare_parameter('wheel_separation', 0.53)  # meters (track width)
        self.declare_parameter('max_rpm', 150.0)
        self.declare_parameter('max_linear_vel', 1.57)  # m/s
        self.declare_parameter('max_angular_vel', 1.57)  # rad/s
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # Calculate max wheel velocity from RPM
        # v = RPM * 2 * pi * r / 60
        self.max_wheel_vel = self.max_rpm * 2 * math.pi * self.wheel_radius / 60
        
        self.get_logger().info(f'Max wheel velocity: {self.max_wheel_vel:.2f} m/s')
        self.get_logger().info(f'Max linear velocity: {self.max_linear_vel:.2f} m/s')
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publishers
        # Wheel velocities in m/s
        self.wheel_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocities',
            10
        )
        
        # Wheel RPMs for motor controller
        self.wheel_rpm_pub = self.create_publisher(
            Int32MultiArray,
            '/wheel_rpm',
            10
        )
        
        # Left and right side velocities (for simpler motor drivers)
        self.left_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/left_wheel_velocities',
            10
        )
        
        self.right_vel_pub = self.create_publisher(
            Float64MultiArray,
            '/right_wheel_velocities',
            10
        )
        
        # Direct motor RPM publisher for Teensy [left_rpm, right_rpm]
        self.motor_rpm_pub = self.create_publisher(
            Float64MultiArray,
            '/motor_rpm',
            10
        )
        
        # Timeout timer
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds
        self.timeout_timer = self.create_timer(0.1, self.timeout_callback)
        
        self.get_logger().info('Skid Steer Controller initialized')
    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist command to wheel velocities
        
        For skid steering:
        - Linear velocity: all wheels same speed
        - Angular velocity: left and right wheels opposite speed
        """
        self.last_cmd_time = self.get_clock().now()
        
        # Clamp velocities
        linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))
        
        # Differential drive kinematics for skid steering
        # v_left = v - w * L/2
        # v_right = v + w * L/2
        left_vel = linear_vel - (angular_vel * self.wheel_separation / 2)
        right_vel = linear_vel + (angular_vel * self.wheel_separation / 2)
        
        # Scale if exceeds max wheel velocity
        max_vel = max(abs(left_vel), abs(right_vel))
        if max_vel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_vel
            left_vel *= scale
            right_vel *= scale
        
        # All 6 wheels - left side and right side
        # Order: front_left, mid_left, rear_left, front_right, mid_right, rear_right
        wheel_velocities = [
            left_vel, left_vel, left_vel,  # Left side
            right_vel, right_vel, right_vel  # Right side
        ]
        
        # Convert to RPM
        wheel_rpms = [int(v * 60 / (2 * math.pi * self.wheel_radius)) for v in wheel_velocities]
        
        # Publish wheel velocities
        vel_msg = Float64MultiArray()
        vel_msg.data = wheel_velocities
        self.wheel_vel_pub.publish(vel_msg)
        
        # Publish wheel RPMs
        rpm_msg = Int32MultiArray()
        rpm_msg.data = wheel_rpms
        self.wheel_rpm_pub.publish(rpm_msg)
        
        # Publish left/right velocities
        left_msg = Float64MultiArray()
        left_msg.data = [left_vel, left_vel, left_vel]
        self.left_vel_pub.publish(left_msg)
        
        right_msg = Float64MultiArray()
        right_msg.data = [right_vel, right_vel, right_vel]
        self.right_vel_pub.publish(right_msg)
        
        # Publish direct left/right RPM for Teensy motor controller
        left_rpm = left_vel * 60 / (2 * math.pi * self.wheel_radius)
        right_rpm = right_vel * 60 / (2 * math.pi * self.wheel_radius)
        motor_rpm_msg = Float64MultiArray()
        motor_rpm_msg.data = [left_rpm, right_rpm]
        self.motor_rpm_pub.publish(motor_rpm_msg)
        
        self.get_logger().debug(
            f'Cmd vel: linear={linear_vel:.2f}, angular={angular_vel:.2f} -> '
            f'left={left_vel:.2f}, right={right_vel:.2f}'
        )
    
    def timeout_callback(self):
        """Stop wheels if no command received recently"""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            # Send zero velocities
            vel_msg = Float64MultiArray()
            vel_msg.data = [0.0] * 6
            self.wheel_vel_pub.publish(vel_msg)
            
            rpm_msg = Int32MultiArray()
            rpm_msg.data = [0] * 6
            self.wheel_rpm_pub.publish(rpm_msg)
            
            # Also send zero to motor_rpm
            motor_rpm_msg = Float64MultiArray()
            motor_rpm_msg.data = [0.0, 0.0]
            self.motor_rpm_pub.publish(motor_rpm_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SkidSteerController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
