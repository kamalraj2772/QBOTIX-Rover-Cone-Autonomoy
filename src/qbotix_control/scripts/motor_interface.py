#!/usr/bin/env python3
"""
Motor Interface for QBotix Rover
Communicates with Teensy 4.1 motor controller via USB Serial
Teensy controls MDS40B Cytron drivers in Anti-Locked Phase mode

Serial Protocol:
  Send: "left_rpm,right_rpm\n"
  Receive: "OK L:xxx R:xxx\n"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float64MultiArray, String
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import threading
import time


class MotorInterface(Node):
    """
    Interface between ROS2 and Teensy 4.1 motor controller
    Converts wheel RPMs to serial commands for Teensy
    """
    
    def __init__(self):
        super().__init__('motor_interface')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')  # Teensy USB serial
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('max_rpm', 150.0)
        self.declare_parameter('command_rate', 20.0)  # Hz
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.command_rate = self.get_parameter('command_rate').value
        
        # State
        self.serial_conn = None
        self.connected = False
        self.simulation_mode = False  # Run without hardware
        self.left_rpm = 0.0
        self.right_rpm = 0.0
        self.last_command_time = time.time()
        
        # Thread lock for serial access
        self.serial_lock = threading.Lock()
        
        # Initialize serial connection (non-blocking)
        self.init_serial()
        
        if not self.connected:
            self.get_logger().warn('Teensy not connected - running in SIMULATION MODE')
            self.get_logger().warn('Motor commands will be logged but not sent to hardware')
            self.simulation_mode = True
        
        # Subscriber for wheel RPMs [fl, ml, rl, fr, mr, rr]
        self.rpm_sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_rpm',
            self.rpm_callback,
            10
        )
        
        # Subscriber for direct left/right RPM commands
        self.lr_rpm_sub = self.create_subscription(
            Float64MultiArray,
            '/motor_rpm',
            self.lr_rpm_callback,
            10
        )
        
        # Publisher for motor feedback
        self.feedback_pub = self.create_publisher(
            String,
            '/motor_feedback',
            10
        )
        
        # Publisher for motor status
        self.status_pub = self.create_publisher(
            String,
            '/motor_status',
            10
        )
        
        # Command timer - send commands at fixed rate
        self.command_timer = self.create_timer(
            1.0 / self.command_rate, 
            self.send_command_callback
        )
        
        # Reconnection timer
        self.reconnect_timer = self.create_timer(2.0, self.check_connection)
        
        self.get_logger().info(f'Motor Interface initialized')
        self.get_logger().info(f'Serial port: {self.serial_port} @ {self.baud_rate} baud')
    
    def init_serial(self):
        """Initialize serial connection to Teensy"""
        try:
            # Try to find Teensy automatically
            if not self.find_teensy():
                self.get_logger().warn(f'Teensy not found, trying {self.serial_port}')
            
            with self.serial_lock:
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=self.timeout
                )
                time.sleep(2)  # Wait for Teensy to reset after connection
                
                # Clear any startup messages
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                
                self.connected = True
                self.get_logger().info(f'Connected to Teensy on {self.serial_port}')
                
                # Send stop command initially
                self.send_stop()
                
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Teensy: {e}')
            self.connected = False
    
    def find_teensy(self):
        """Auto-detect Teensy serial port"""
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            # Teensy VID:PID is 16C0:0483
            if port.vid == 0x16C0 and port.pid == 0x0483:
                self.serial_port = port.device
                self.get_logger().info(f'Found Teensy at {port.device}')
                return True
            # Also check description
            if 'teensy' in port.description.lower():
                self.serial_port = port.device
                self.get_logger().info(f'Found Teensy at {port.device}')
                return True
        
        return False
    
    def check_connection(self):
        """Periodically check and reconnect if needed"""
        if not self.connected:
            self.get_logger().info('Attempting to reconnect to Teensy...')
            self.init_serial()
    
    def rpm_callback(self, msg):
        """
        Handle wheel RPM commands [fl, ml, rl, fr, mr, rr]
        For skid steering, left side = fl, ml, rl and right side = fr, mr, rr
        """
        if len(msg.data) != 6:
            self.get_logger().warn('Expected 6 RPM values')
            return
        
        # Average RPMs for each side (should be same for skid steering)
        self.left_rpm = float(msg.data[0] + msg.data[1] + msg.data[2]) / 3.0
        self.right_rpm = float(msg.data[3] + msg.data[4] + msg.data[5]) / 3.0
        
        # Clamp to max RPM
        self.left_rpm = max(-self.max_rpm, min(self.max_rpm, self.left_rpm))
        self.right_rpm = max(-self.max_rpm, min(self.max_rpm, self.right_rpm))
        
        self.last_command_time = time.time()
    
    def lr_rpm_callback(self, msg):
        """Handle direct left/right RPM commands [left, right]"""
        if len(msg.data) != 2:
            self.get_logger().warn('Expected 2 RPM values [left, right]')
            return
        
        self.left_rpm = float(msg.data[0])
        self.right_rpm = float(msg.data[1])
        
        # Clamp to max RPM
        self.left_rpm = max(-self.max_rpm, min(self.max_rpm, self.left_rpm))
        self.right_rpm = max(-self.max_rpm, min(self.max_rpm, self.right_rpm))
        
        self.last_command_time = time.time()
    
    def send_command_callback(self):
        """Send motor commands at fixed rate"""
        # Safety timeout - stop if no commands received for 1 second
        if time.time() - self.last_command_time > 1.0:
            self.left_rpm = 0.0
            self.right_rpm = 0.0
        
        self.send_motor_command(self.left_rpm, self.right_rpm)
    
    def send_motor_command(self, left_rpm, right_rpm):
        """Send RPM command to Teensy"""
        # Simulation mode - just log commands
        if self.simulation_mode:
            if abs(left_rpm) > 0.1 or abs(right_rpm) > 0.1:
                self.get_logger().debug(f'[SIM] Motor cmd: L={left_rpm:.1f} R={right_rpm:.1f} RPM')
            return
        
        if not self.connected:
            return
        
        try:
            with self.serial_lock:
                # Format: "left_rpm,right_rpm\n"
                cmd = f"{left_rpm:.1f},{right_rpm:.1f}\n"
                self.serial_conn.write(cmd.encode())
                
                # Read response (non-blocking)
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode().strip()
                    
                    # Publish feedback
                    feedback_msg = String()
                    feedback_msg.data = response
                    self.feedback_pub.publish(feedback_msg)
                    
                    if response.startswith("ERR"):
                        self.get_logger().warn(f'Teensy error: {response}')
                        
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
            self.connected = False
        except Exception as e:
            self.get_logger().error(f'Command error: {e}')
    
    def send_stop(self):
        """Send emergency stop command"""
        if not self.connected:
            return
        
        try:
            with self.serial_lock:
                self.serial_conn.write(b"STOP\n")
                self.left_rpm = 0.0
                self.right_rpm = 0.0
                self.get_logger().info('Stop command sent')
        except Exception as e:
            self.get_logger().error(f'Failed to send stop: {e}')
    
    def get_status(self):
        """Request status from Teensy"""
        if not self.connected:
            return
        
        try:
            with self.serial_lock:
                self.serial_conn.write(b"STATUS\n")
                time.sleep(0.05)
                if self.serial_conn.in_waiting > 0:
                    response = self.serial_conn.readline().decode().strip()
                    
                    status_msg = String()
                    status_msg.data = response
                    self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to get status: {e}')
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down motor interface...')
        
        # Send stop command
        self.send_stop()
        
        # Close serial connection
        with self.serial_lock:
            if self.serial_conn is not None and self.serial_conn.is_open:
                self.serial_conn.close()
                self.get_logger().info('Serial connection closed')


def main(args=None):
    rclpy.init(args=args)
    node = MotorInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
