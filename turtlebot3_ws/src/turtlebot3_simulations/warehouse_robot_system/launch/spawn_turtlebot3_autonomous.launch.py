#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Launch configuration variables
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'x_pose', default_value='0.0',
            description='x position of the robot'),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='y position of the robot'),

        # Spawn TurtleBot3 in Gazebo (without wall following controller)
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description',
                      '-name', 'turtlebot3_burger',
                      '-x', x_pose,
                      '-y', y_pose,
                      '-z', '0.01'],
            output='screen'),

        # Bridge for Gazebo communication (without wall following)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                'clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                'joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                'odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                'tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                'cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist',
                'imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                'scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '--ros-args', '-r', '/tf:=/tf_gz'
            ],
            output='screen'),

        # Camera red detector (if needed)
        Node(
            package='turtlebot3_gazebo',
            executable='camera_red_detector',
            name='camera_red_detector',
            output='screen'),
    ])