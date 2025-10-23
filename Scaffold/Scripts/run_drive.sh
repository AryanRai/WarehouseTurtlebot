#!/bin/bash
# Run the autonomous drive node with optional driving mode selection

cd turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger

# Check if mode argument provided
if [ "$1" == "center" ] || [ "$1" == "c" ]; then
    echo "Starting autonomous drive node in CENTERING mode..."
    ros2 run turtlebot3_gazebo turtlebot3_drive --ros-args -p use_centering:=true
elif [ "$1" == "right_wall" ] || [ "$1" == "rw" ]; then
    echo "Starting autonomous drive node in RIGHT WALL FOLLOW mode..."
    ros2 run turtlebot3_gazebo turtlebot3_drive --ros-args -p use_centering:=false
else
    echo "Starting autonomous drive node in RIGHT WALL FOLLOW mode (default)..."
    echo "Usage: ./run_drive.sh [right_wall|center]"
    echo "  right_wall (rw) - Follow right wall only (default, more reliable)"
    echo "  center (c)      - Center between walls (experimental)"
    echo ""
    ros2 run turtlebot3_gazebo turtlebot3_drive
fi
