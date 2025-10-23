#!/bin/bash
# Simple teleop script for TurtleBot3 control

echo "🎮 Starting TurtleBot3 Keyboard Teleop"
echo "======================================"
echo ""
echo "Controls:"
echo "   w/x : increase/decrease linear velocity"
echo "   a/d : increase/decrease angular velocity"
echo "   space/s : force stop"
echo "   CTRL+C : quit"
echo ""

cd "$(dirname "$0")/../turtlebot3_ws"
source install/setup.bash

ros2 run turtlebot3_teleop teleop_keyboard