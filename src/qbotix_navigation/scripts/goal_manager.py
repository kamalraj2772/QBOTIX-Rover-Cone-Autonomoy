#!/usr/bin/env python3
"""
Goal Manager for QBotix Rover
Manages navigation goals and cone tracking
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import math


class GoalManager(Node):
    """
    Manages navigation goals and provides goal tracking
    """
    
    def __init__(self):
        super().__init__('goal_manager')
        
        # Parameters
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('auto_retry', True)
        self.declare_parameter('max_retries', 3)
        
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.auto_retry = self.get_parameter('auto_retry').value
        self.max_retries = self.get_parameter('max_retries').value
        
        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Current goal
        self.current_goal = None
        self.goal_active = False
        self.retry_count = 0
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.cancel_sub = self.create_subscription(
            Bool,
            '/cancel_goal',
            self.cancel_callback,
            10
        )
        
        # Publishers
        self.goal_status_pub = self.create_publisher(
            Bool,
            '/goal_reached',
            10
        )
        
        self.get_logger().info('Goal Manager initialized')
        self.get_logger().info('Waiting for navigation action server...')
        
        # Wait for action server
        if self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Navigation action server available')
        else:
            self.get_logger().warn('Navigation action server not available')
    
    def goal_callback(self, msg):
        """Handle new goal"""
        self.current_goal = msg
        self.retry_count = 0
        self.send_goal(msg)
    
    def cancel_callback(self, msg):
        """Cancel current goal"""
        if msg.data and self.goal_active:
            self.get_logger().info('Cancelling current goal')
            # Cancel would be implemented here
            self.goal_active = False
    
    def send_goal(self, pose):
        """Send navigation goal"""
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Navigation server not ready')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(
            f'Sending goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )
        
        self.goal_active = True
        
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.goal_active = False
            return
        
        self.get_logger().info('Goal accepted')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        # Distance remaining, ETA, etc.
        pass
    
    def result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        self.goal_active = False
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached!')
            msg = Bool()
            msg.data = True
            self.goal_status_pub.publish(msg)
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            
            # Retry if enabled
            if self.auto_retry and self.retry_count < self.max_retries:
                self.retry_count += 1
                self.get_logger().info(f'Retrying... (attempt {self.retry_count})')
                if self.current_goal:
                    self.send_goal(self.current_goal)


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
