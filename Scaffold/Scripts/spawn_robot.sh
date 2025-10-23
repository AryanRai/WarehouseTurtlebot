#!/bin/bash
# Manually spawn TurtleBot3 in Gazebo

cd turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "Spawning TurtleBot3 at position (0, 0)..."
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0
