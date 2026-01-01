#!/usr/bin/env python3
"""
Complete Nav2 Navigation Launch for QBotix Rover
Includes ALL components: costmaps + navigation servers + lifecycle management
Also includes TF broadcaster, odometry publisher, and goal manager
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('qbotix_navigation')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Lifecycle nodes - ALL nodes that need lifecycle management
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'smoother_server',
        'velocity_smoother',
    ]
    
    # Remap rules for costmaps
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # Configure parameters with use_sim_time
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )
    
    return LaunchDescription([
        # Environment variable for logging
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
            description='Nav2 parameters file'
        ),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        
        # ========================================
        # TF BROADCASTER (map->odom->base_link)
        # ========================================
        Node(
            package='qbotix_navigation',
            executable='tf_broadcaster.py',
            name='tf_broadcaster',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # ========================================
        # ODOMETRY PUBLISHER (ZED odom -> /odom)
        # ========================================
        Node(
            package='qbotix_navigation',
            executable='odom_publisher.py',
            name='odom_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_tf': False,  # TF handled by tf_broadcaster
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',  # Nav2 expects base_footprint in TF tree
                'zed_odom_topic': '/zed/zed_node/odom'
            }]
        ),
        
        # ========================================
        # GOAL MANAGER (navigation goal handling)
        # ========================================
        Node(
            package='qbotix_navigation',
            executable='goal_manager.py',
            name='goal_manager',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'goal_tolerance': 0.5,
                'auto_retry': True,
                'max_retries': 3
            }]
        ),
        
        # ========================================
        # CONTROLLER SERVER (includes local costmap)
        # ========================================
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
        ),
        
        # ========================================
        # PLANNER SERVER (includes global costmap)
        # ========================================
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings
        ),
        
        # ========================================
        # BEHAVIOR SERVER
        # ========================================
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings
        ),
        
        # ========================================
        # BT NAVIGATOR
        # ========================================
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings
        ),
        
        # ========================================
        # SMOOTHER SERVER
        # ========================================
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings
        ),
        
        # ========================================
        # VELOCITY SMOOTHER
        # ========================================
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params],
            remappings=remappings + [
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel')
            ]
        ),
        
        # ========================================
        # LIFECYCLE MANAGER
        # ========================================
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
                'bond_timeout': 4.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 10.0,
            }]
        ),
    ])
