#!/usr/bin/env python3
"""
Cone Registration Controller Node
==================================
Handles keyboard input for cone registration control.

USAGE:
  - The rover automatically registers the cone position on first detection
  - If an obstacle hides the cone, the rover continues to the registered position
  - Press Ctrl+Q to RESET and re-register the cone position (reads new position)
  - Press Ctrl+L to LOCK the current cone position (ignores new detections)
  - Press Ctrl+U to UNLOCK and allow position updates
  - Press Ctrl+S to STOP navigation
  - Press Ctrl+R to RESUME navigation

FEATURES:
  - Persistent cone position memory
  - Navigation through occlusions (dynamic/static obstacles)
  - Manual reset for re-reading cone position
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
import sys
import termios
import tty
import select
import threading


class ConeRegistrationController(Node):
    """
    Keyboard controller for cone registration system.
    Runs in terminal and listens for key commands.
    """
    
    def __init__(self):
        super().__init__('cone_registration_controller')
        
        # Publishers
        self.reset_pub = self.create_publisher(Bool, '/reset_cone_registration', 10)
        self.lock_pub = self.create_publisher(Bool, '/lock_cone_position', 10)
        self.enable_pub = self.create_publisher(Bool, '/enable_auto_navigation', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/cone_nav_status',
            self.status_callback,
            10
        )
        
        self.cone_pose_sub = self.create_subscription(
            PoseStamped,
            '/cone_pose_map',
            self.cone_pose_callback,
            10
        )
        
        # State
        self.current_status = "UNKNOWN"
        self.current_cone_position = None
        self.is_locked = False
        self.navigation_enabled = True
        
        # Print instructions
        self.print_instructions()
        
        # Start keyboard listener in separate thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Status display timer
        self.create_timer(2.0, self.display_status)
        
        self.get_logger().info('Cone Registration Controller started')
    
    def print_instructions(self):
        """Print keyboard instructions"""
        print("\n" + "="*60)
        print("       CONE REGISTRATION CONTROLLER")
        print("="*60)
        print("\n  KEYBOARD COMMANDS:")
        print("  -------------------")
        print("  Ctrl+Q  : RESET - Clear registration, read new cone position")
        print("  Ctrl+L  : LOCK  - Lock current position (ignore new detections)")
        print("  Ctrl+U  : UNLOCK - Allow position updates from detections")
        print("  Ctrl+S  : STOP  - Stop navigation")
        print("  Ctrl+R  : RESUME - Resume navigation")
        print("  Ctrl+C  : EXIT  - Exit controller")
        print("\n" + "="*60)
        print("\n  CURRENT STATUS: Waiting for cone detection...\n")
    
    def status_callback(self, msg):
        """Update current navigation status"""
        self.current_status = msg.data
    
    def cone_pose_callback(self, msg):
        """Track current cone position"""
        self.current_cone_position = (
            msg.pose.position.x,
            msg.pose.position.y
        )
    
    def display_status(self):
        """Display current status periodically"""
        lock_status = "🔒 LOCKED" if self.is_locked else "🔓 UNLOCKED"
        nav_status = "▶️ ACTIVE" if self.navigation_enabled else "⏸️ PAUSED"
        
        if self.current_cone_position:
            x, y = self.current_cone_position
            print(f"\r  📍 Cone: ({x:.2f}, {y:.2f}) | {lock_status} | {nav_status} | {self.current_status}    ", end='', flush=True)
        else:
            print(f"\r  🔍 Searching for cone... | {lock_status} | {nav_status}    ", end='', flush=True)
    
    def keyboard_listener(self):
        """Listen for keyboard input in separate thread"""
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while self.running and rclpy.ok():
                # Check if input is available
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    
                    # Check for Ctrl key combinations
                    if ord(ch) == 17:  # Ctrl+Q
                        self.handle_reset()
                    elif ord(ch) == 12:  # Ctrl+L
                        self.handle_lock()
                    elif ord(ch) == 21:  # Ctrl+U
                        self.handle_unlock()
                    elif ord(ch) == 19:  # Ctrl+S
                        self.handle_stop()
                    elif ord(ch) == 18:  # Ctrl+R
                        self.handle_resume()
                    elif ord(ch) == 3:  # Ctrl+C
                        self.running = False
                        break
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def handle_reset(self):
        """Reset cone registration - will read new position"""
        msg = Bool()
        msg.data = True
        self.reset_pub.publish(msg)
        self.is_locked = False
        print("\n\n  🔄 RESET: Cone registration cleared! Waiting for new detection...\n")
        self.get_logger().info('Reset cone registration - waiting for new detection')
    
    def handle_lock(self):
        """Lock current cone position"""
        msg = Bool()
        msg.data = True
        self.lock_pub.publish(msg)
        self.is_locked = True
        if self.current_cone_position:
            x, y = self.current_cone_position
            print(f"\n\n  🔒 LOCKED: Cone position fixed at ({x:.2f}, {y:.2f})\n")
        else:
            print("\n\n  ⚠️  Cannot lock - no cone position registered yet!\n")
        self.get_logger().info('Lock cone position')
    
    def handle_unlock(self):
        """Unlock to allow position updates"""
        msg = Bool()
        msg.data = False
        self.lock_pub.publish(msg)
        self.is_locked = False
        print("\n\n  🔓 UNLOCKED: Cone position can be updated from detections\n")
        self.get_logger().info('Unlock cone position')
    
    def handle_stop(self):
        """Stop navigation"""
        msg = Bool()
        msg.data = False
        self.enable_pub.publish(msg)
        self.navigation_enabled = False
        print("\n\n  ⏸️  STOPPED: Navigation paused\n")
        self.get_logger().info('Stop navigation')
    
    def handle_resume(self):
        """Resume navigation"""
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
        self.navigation_enabled = True
        print("\n\n  ▶️  RESUMED: Navigation active\n")
        self.get_logger().info('Resume navigation')


def main(args=None):
    rclpy.init(args=args)
    node = ConeRegistrationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        # Restore terminal
        import os
        os.system('stty sane')
        print("\n\nController exited.\n")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
