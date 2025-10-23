#!/bin/bash
# Run teleop keyboard control

cd turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

echo "Starting teleop keyboard control..."
echo "Use WASD to control the robot"
ros2 run turtlebot3_teleop teleop_keyboard
