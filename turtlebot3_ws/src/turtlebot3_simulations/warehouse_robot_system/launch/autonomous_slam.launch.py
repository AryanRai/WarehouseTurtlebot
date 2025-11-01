#!/usr/bin/env python3
"""
Autonomous SLAM Exploration Launch File
Launches SLAM Toolbox and autonomous exploration node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('warehouse_robot_system')
    
    # Declare arguments
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug visualization'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # SLAM Toolbox configuration
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    
    # Include SLAM Toolbox mapping launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'slam_toolbox_mapping.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )
    
    # Autonomous SLAM exploration node
    autonomous_slam_node = Node(
        package='warehouse_robot_system',
        executable='autonomous_slam_node',
        name='autonomous_slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'debug': LaunchConfiguration('debug')
        }]
    )
    
    # Home position marker (static TF at 0,0)
    home_position_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='home_position_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'home_position'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        debug_arg,
        use_sim_time_arg,
        slam_launch,
        autonomous_slam_node,
        home_position_tf
    ])
