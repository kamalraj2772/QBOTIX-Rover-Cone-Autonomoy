"""
Control Launch File for QBotix Rover
Launches motor control and velocity processing
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Parameters
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    max_rpm = LaunchConfiguration('max_rpm')
    
    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.10',
        description='Wheel radius in meters'
    )
    
    declare_wheel_separation = DeclareLaunchArgument(
        'wheel_separation',
        default_value='0.53',
        description='Wheel separation (track width) in meters'
    )
    
    declare_max_rpm = DeclareLaunchArgument(
        'max_rpm',
        default_value='150.0',
        description='Maximum wheel RPM'
    )
    
    # Skid steer controller
    skid_steer_controller = Node(
        package='qbotix_control',
        executable='skid_steer_controller.py',
        name='skid_steer_controller',
        output='screen',
        parameters=[{
            'wheel_radius': wheel_radius,
            'wheel_separation': wheel_separation,
            'max_rpm': max_rpm,
            'max_linear_vel': 1.57,
            'max_angular_vel': 1.57
        }]
    )
    
    # Motor interface
    motor_interface = Node(
        package='qbotix_control',
        executable='motor_interface.py',
        name='motor_interface',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',  # Teensy uses ACM port
            'baud_rate': 115200,
            'max_rpm': max_rpm,
            'command_rate': 20.0
        }]
    )
    
    # Velocity smoother (optional - Nav2 has its own smoother)
    velocity_smoother = Node(
        package='qbotix_control',
        executable='velocity_smoother.py',
        name='velocity_smoother_control',
        output='screen',
        parameters=[{
            'max_linear_accel': 0.5,
            'max_angular_accel': 1.0,
            'max_linear_vel': 0.8,
            'max_angular_vel': 1.0,
            'smoothing_rate': 50.0
        }],
        remappings=[
            ('/cmd_vel_raw', '/cmd_vel_nav'),
            ('/cmd_vel', '/cmd_vel')
        ]
    )
    
    return LaunchDescription([
        declare_wheel_radius,
        declare_wheel_separation,
        declare_max_rpm,
        skid_steer_controller,
        motor_interface,
    ])
