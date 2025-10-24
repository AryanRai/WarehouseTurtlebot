#!/usr/bin/env python3
#
# MTRX3760 2025 Project 2: Warehouse Robot
# Launch file to spawn TurtleBot3 for autonomous SLAM (without wall following)
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the urdf file
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0')

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Initial x position')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Initial y position')
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Initial yaw (rad)')

    # Spawn robot into Gazebo
    start_gazebo_ros_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', TURTLEBOT3_MODEL,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-Y', yaw
        ],
        output='screen',
    )

    # ROS <-> Gazebo bridge (twist, odom, laser, etc.)
    bridge_params = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'params',
        model_folder + '_bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Camera image bridge (gz -> ROS /camera/image_raw) - only for non-burger models
    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    # Camera red detector node
    start_red_detector_cmd = Node(
        package='turtlebot3_gazebo',
        executable='camera_red_detector',
        name='camera_red_detector',
        output='screen',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'min_area_frac': 0.02,
            'publish_debug': True
        }],
    )

    # NOTE: We do NOT start turtlebot3_drive node here
    # The autonomous_slam_controller will handle all movement commands

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_yaw_cmd)

    # Add actions
    ld.add_action(start_gazebo_ros_spawner_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    
    # Add camera bridge and detector only for non-burger models
    if TURTLEBOT3_MODEL != 'burger':
        ld.add_action(start_gazebo_ros_image_bridge_cmd)
        ld.add_action(start_red_detector_cmd)

    return ld
