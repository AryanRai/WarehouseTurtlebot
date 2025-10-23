#!/bin/bash
# Launch TurtleBot3 with Cartographer SLAM for maze mapping

echo "=========================================="
echo "TurtleBot3 SLAM Mapper with Cartographer"
echo "=========================================="
echo ""

# Set up environment
export TURTLEBOT3_MODEL=burger
cd turtlebot3_ws
source install/setup.bash
cd ..

# Check if Gazebo is already running
if pgrep -f "gz sim" > /dev/null; then
    echo "✅ Gazebo is already running"
else
    echo "❌ Gazebo is not running!"
    echo "Please run ./launch_mgen.sh first to start the maze simulation"
    exit 1
fi

echo ""
echo "🔧 Starting robot_state_publisher (TF publisher)..."
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True &
RSP_PID=$!

sleep 2

echo ""
echo "🗺️  Starting Cartographer SLAM (Optimized for Maze)..."
echo ""

# Launch Cartographer SLAM with optimized configuration
ros2 launch turtlebot3_cartographer cartographer.launch.py \
    use_sim_time:=True \
    configuration_basename:=turtlebot3_lds_2d_optimized.lua &
CARTOGRAPHER_PID=$!

echo "⏳ Waiting for Cartographer to initialize..."
sleep 5

# Check if Cartographer started successfully
if ! ps -p $CARTOGRAPHER_PID > /dev/null 2>&1; then
    echo "❌ Cartographer failed to start!"
    exit 1
fi

echo ""
echo "🤖 Starting path visualizer..."
ros2 run turtlebot3_gazebo path_visualizer &
PATH_VIS_PID=$!

sleep 2

echo ""
echo "🎮 Starting autonomous drive node..."
ros2 run turtlebot3_gazebo turtlebot3_drive &
DRIVE_PID=$!

sleep 2

echo ""
echo "📺 Launching RViz2 with SLAM configuration..."
rviz2 -d slam_config.rviz &
RVIZ_PID=$!

sleep 2

echo ""
echo "✅ SLAM System Launched Successfully!"
echo ""
echo "📊 What's Running:"
echo "   - Cartographer SLAM (building map)"
echo "   - Path Visualizer (tracking trajectory)"
echo "   - TurtleBot3 Drive (autonomous navigation)"
echo ""
echo "🖥️  RViz2 Tips:"
echo "   1. RViz2 should have opened automatically"
echo "   2. Add these displays if not visible:"
echo "      - Map (topic: /map) - shows the SLAM map"
echo "      - Path (topic: /robot_path) - shows trajectory"
echo "      - LaserScan (topic: /scan) - shows lidar data"
echo "      - RobotModel - shows the robot"
echo "   3. Set Fixed Frame to: 'map'"
echo ""
echo "💾 To Save the Map:"
echo "   Run in a new terminal:"
echo "   ./save_map.sh"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo ""

# Wait for user interrupt
wait
