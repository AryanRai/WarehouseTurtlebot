#!/bin/bash
# Kill all Gazebo and TurtleBot3 processes

echo "Stopping all Gazebo and TurtleBot3 processes..."

# Kill Gazebo
pkill -9 -f "gz sim"
pkill -9 gzserver
pkill -9 gzclient
pkill -9 gazebo

# Kill TurtleBot3 nodes
pkill -9 -f turtlebot3_drive
pkill -9 -f turtlebot3_teleop
pkill -9 -f ros_gz_bridge
pkill -9 -f parameter_bridge
pkill -9 -f robot_state_publisher
pkill -9 -f path_visualizer

# Kill SLAM nodes
pkill -9 -f cartographer
pkill -9 -f "cartographer.launch"

# Kill RViz
pkill -9 -f rviz2

# Wait a moment
sleep 1

echo "All processes stopped!"
echo ""
echo "You can now run ./run_gazebo.sh to start fresh."
