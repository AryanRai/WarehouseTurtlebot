#!/usr/bin/env python3
"""
SLAM Toolbox Localization Launch File
Launches SLAM Toolbox in localization mode for delivery operations
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('warehouse_robot_system')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map_file', default='/tmp/warehouse_map.yaml')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_dir, 'config', 'slam_toolbox_localization_params.yaml')
    )
    
    # SLAM Toolbox localization node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,
            }
        ],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value='/tmp/warehouse_map.yaml',
            description='Full path to map file (without extension)'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'slam_toolbox_localization_params.yaml'),
            description='Full path to SLAM Toolbox localization parameters file'
        ),
        slam_toolbox_node,
    ])
