#!/bin/bash
# MTRX3760 2025 Project 2: Launch Warehouse Shelves World
# 
# This script launches the warehouse environment with shelving units
# based on the project specifications (2.3m x 2.3m with 4 shelves)

echo "=========================================="
echo "  MTRX3760 Warehouse Shelves Launcher    "
echo "=========================================="
echo ""

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Get the script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$SCRIPT_DIR/turtlebot3_ws"

# Source ROS 2 workspace
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "✓ Sourcing workspace: $WORKSPACE_DIR"
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "⚠ Warning: Workspace not built. Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
fi

echo ""
echo "Configuration:"
echo "  - World: warehouse_shelves.world"
echo "  - Dimensions: 2.3m x 2.3m"
echo "  - Shelves: 4 horizontal units (1.15m wide)"
echo "  - Robot model: TurtleBot3 Burger"
echo "  - Starting position: Origin (0, 0)"
echo ""

# Check if world file exists
WORLD_FILE="$WORKSPACE_DIR/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_shelves.world"
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ ERROR: World file not found at:"
    echo "   $WORLD_FILE"
    exit 1
fi

echo "✓ World file found"
echo ""
echo "Launching Gazebo with warehouse environment..."
echo "Press Ctrl+C to stop"
echo ""
echo "=========================================="

# Launch Gazebo with the warehouse world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=$WORLD_FILE