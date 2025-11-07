#!/bin/bash
# Spawn TurtleBot3 in Gazebo (based on reference script)

echo " Spawning TurtleBot3 in Gazebo"
echo "================================"
echo ""

# Set up environment
export TURTLEBOT3_MODEL=burger
cd "$(dirname "$0")/../turtlebot3_ws"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo " Workspace not built! Please run 'colcon build' first."
    exit 1
fi

source install/setup.bash

# Check if Gazebo is running
if ! pgrep -f "gz sim" > /dev/null; then
    echo " Gazebo is not running!"
    echo "Please start Gazebo first with:"
    echo "   ./launch_mgen.sh"
    echo "   or"
    echo "   ./scripts/run_full_slam_demo.sh"
    exit 1
fi

echo " Gazebo is running"
echo ""

# Get spawn position from arguments or use defaults
X_POSE=${1:-1.0}
Y_POSE=${2:--1.0}

echo " Spawning TurtleBot3 at position ($X_POSE, $Y_POSE)..."

# Start robot_state_publisher first
echo "Starting robot_state_publisher..."
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True &
RSP_PID=$!
sleep 2

# Spawn the robot
echo "Spawning robot..."
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=$X_POSE y_pose:=$Y_POSE

echo ""
echo " TurtleBot3 spawned successfully!"
echo ""
echo " Next steps:"
echo "   • Run SLAM: ./scripts/run_slam_sim.sh"
echo "   • Control robot: ./scripts/run_teleop.sh"
echo "   • Test warehouse system: ros2 run warehouse_robot_system warehouse_robot_main"