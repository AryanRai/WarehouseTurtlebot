#!/bin/bash
# Launch fixed warehouse world with TurtleBot3

echo "=========================================="
echo "Mini Warehouse Launcher"
echo "=========================================="
echo ""
echo "World: warehouse_world (2.3m x 2.3m floor, 4 shelves)"
echo "Physics: ODE, 0.001s step, 1000Hz real-time update"
echo ""

# Path to the warehouse world SDF
WORLD_FILE="turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_world.world"

if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ World file not found at: $WORLD_FILE"
    echo "   Make sure warehouse_world.world exists."
    exit 1
fi

# Set up environment for TurtleBot3
export TURTLEBOT3_MODEL=burger

# Source ROS2 workspace
cd turtlebot3_ws
source install/setup.bash

# Set up Gazebo model paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/src/turtlebot3_simulations/turtlebot3_gazebo/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(pwd)/src/turtlebot3_simulations/turtlebot3_gazebo

echo ""
echo "🚀 Launching TurtleBot3 Gazebo with warehouse world..."
echo ""

# Use the TurtleBot3 Gazebo launch file with custom world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=$(pwd)/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_world.world x_pose:=0.0 y_pose:=0.0 &
GAZEBO_PID=$!

echo "⏳ Waiting for Gazebo to initialize..."
sleep 8

# Check if Gazebo is still running
if ! ps -p $GAZEBO_PID > /dev/null; then
    echo "❌ Gazebo failed to start!"
    exit 1
fi

# Give it time to fully load
sleep 5

echo ""
echo "✅ Warehouse world launched successfully!"
echo ""
echo "📦 World Info:"
echo "   • Floor: 2.3 m x 2.3 m"
echo "   • Wall thickness: 0.05 m"
echo "   • Wall / shelf height: 1.0 m"
echo "   • Shelves at y = ±0.69, ±0.23 m"
echo "   • Gravity: 0 0 -9.81"
echo "   • Physics step: 0.001 s (ODE)"
echo "   File: ${WORLD_FILE}"
echo ""
echo "🤖 Robot:"
echo "   • Model: TurtleBot3 Burger"
echo "   • Spawn pose: (0.0, 0.0, ~0.01)"
echo ""
echo "🎮 Next Steps:"
echo "   1. Open a new terminal"
echo "   2. cd to turtlebot3_ws and source install/setup.bash"
echo "   3. Run teleop / nav stack / your controller script"
echo ""
echo "💡 Tip:"
echo "   Use 'gz topic -l' and 'ros2 topic list' to inspect sensors."
echo ""
echo "Press Ctrl+C to stop this script."
echo "Gazebo and the robot will keep running until you kill them (./kill_all.sh recommended)."
echo ""

# Keep script running so processes don't get auto-killed
wait
