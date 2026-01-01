"""
ZED2i Camera Launch File for QBotix Rover
Launches ZED ROS2 wrapper with appropriate settings
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_qbotix_rover = get_package_share_directory('qbotix_rover')
    
    # Get ZED wrapper package
    try:
        zed_wrapper_dir = get_package_share_directory('zed_wrapper')
        zed_available = True
    except:
        zed_available = False
        print("WARNING: zed_wrapper package not found. Install zed-ros2-wrapper.")
    
    # Launch arguments
    camera_model = LaunchConfiguration('camera_model')
    camera_name = LaunchConfiguration('camera_name')
    
    declare_camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='Camera model (zed2, zed2i, zedx, etc.)'
    )
    
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='zed',
        description='Camera name for namespacing'
    )
    
    # ZED config file
    zed_config = os.path.join(pkg_qbotix_rover, 'config', 'zed_config.yaml')
    
    actions = [
        declare_camera_model,
        declare_camera_name,
    ]
    
    if zed_available:
        # Include ZED launch file with our config
        zed_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
            ]),
            launch_arguments={
                'camera_model': camera_model,
                'camera_name': camera_name,
                'config_path': zed_config,
                # Disable ZED TF publishing - URDF handles this
                'publish_tf': 'false',
                'publish_map_tf': 'false',
            }.items()
        )
        actions.append(LogInfo(msg='Starting ZED2i camera with custom config...'))
        actions.append(zed_launch)
    else:
        # Placeholder message
        actions.append(LogInfo(msg='WARNING: ZED wrapper not available - camera will not start'))
    
    return LaunchDescription(actions)
