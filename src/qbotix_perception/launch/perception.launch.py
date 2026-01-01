"""
Perception Launch File for QBotix Rover
Launches YOLO cone detection and depth processing
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Parameters
    model_path = LaunchConfiguration('model_path')
    confidence = LaunchConfiguration('confidence')
    
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/home/qbotixrover/Documents/small.pt',
        description='Path to YOLO model file'
    )
    
    declare_confidence = DeclareLaunchArgument(
        'confidence',
        default_value='0.5',
        description='Detection confidence threshold'
    )
    
    # Cone detector node
    cone_detector = Node(
        package='qbotix_perception',
        executable='cone_detector.py',
        name='cone_detector',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'confidence_threshold': confidence,
            'image_topic': '/zed/zed_node/rgb/color/rect/image',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'camera_info_topic': '/zed/zed_node/rgb/color/rect/camera_info',
            'detection_rate': 10.0,
            'max_detection_distance': 10.0,
            'min_detection_distance': 0.3
        }]
    )
    
    # Target publisher node
    target_publisher = Node(
        package='qbotix_perception',
        executable='target_publisher.py',
        name='target_publisher',
        output='screen',
        parameters=[{
            'goal_offset_distance': 0.5,
            'min_goal_update_distance': 0.3,
            'auto_navigate': True,
            'smooth_goal': True,
            'smoothing_window': 5
        }]
    )
    
    # Depth processor node
    depth_processor = Node(
        package='qbotix_perception',
        executable='depth_processor.py',
        name='depth_processor',
        output='screen',
        parameters=[{
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'min_obstacle_height': 0.15,
            'max_obstacle_height': 2.0
        }]
    )
    
    return LaunchDescription([
        declare_model_path,
        declare_confidence,
        cone_detector,
        target_publisher,
        depth_processor
    ])
