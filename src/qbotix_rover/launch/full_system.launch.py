"""
Full System Launch File for QBotix Rover
Launches all components for autonomous cone navigation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_qbotix_rover = get_package_share_directory('qbotix_rover')
    pkg_qbotix_navigation = get_package_share_directory('qbotix_navigation')
    pkg_qbotix_perception = get_package_share_directory('qbotix_perception')
    pkg_qbotix_control = get_package_share_directory('qbotix_control')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    model_path = LaunchConfiguration('model_path')
    enable_rviz = LaunchConfiguration('enable_rviz')
    enable_perception = LaunchConfiguration('enable_perception')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_model_path = DeclareLaunchArgument(
        'model_path',
        default_value='/home/qbotixrover/Documents/small.pt',
        description='Path to YOLO model'
    )
    
    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )
    
    declare_enable_perception = DeclareLaunchArgument(
        'enable_perception',
        default_value='true',
        description='Enable YOLO perception'
    )
    
    # Log startup
    log_startup = LogInfo(msg='\n========================================\nStarting QBotix Rover Full System...\n========================================')
    
    # 1. Robot description and state publisher
    rover_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_qbotix_rover, 'launch', 'rover_bringup.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # 2. ZED Camera (with delay to let robot description start)
    zed_camera = TimerAction(
        period=2.0,
        actions=[
            LogInfo(msg='[2s] Starting ZED Camera...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_qbotix_rover, 'launch', 'zed_camera.launch.py')
                ]),
                launch_arguments={
                    'camera_model': 'zed2i',
                    'camera_name': 'zed',
                }.items()
            )
        ]
    )
    
    # 3. Control nodes (with delay for ZED to initialize)
    control = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg='[5s] Starting Control System...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_qbotix_control, 'launch', 'control.launch.py')
                ]),
                launch_arguments={
                    'wheel_radius': '0.10',
                    'wheel_separation': '0.53',
                    'max_rpm': '150.0',
                }.items()
            )
        ]
    )
    
    # 4. Navigation (with delay for ZED and control to initialize)
    navigation = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg='[8s] Starting Navigation Stack...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_qbotix_navigation, 'launch', 'navigation.launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'autostart': 'true',
                }.items()
            )
        ]
    )
    
    # 5. Perception (YOLO cone detection - with longer delay for full system)
    perception = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg='[12s] Starting Perception System...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(pkg_qbotix_perception, 'launch', 'perception.launch.py')
                ]),
                launch_arguments={
                    'model_path': model_path,
                    'confidence': '0.5',
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_model_path,
        declare_enable_rviz,
        declare_enable_perception,
        log_startup,
        rover_bringup,
        zed_camera,
        control,
        navigation,
        perception
    ])
