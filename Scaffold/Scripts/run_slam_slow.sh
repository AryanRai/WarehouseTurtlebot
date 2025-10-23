#!/bin/bash
# Launch TurtleBot3 with SLAM at REDUCED SPEED for better mapping
# This is the BEST option for clean maps - robot moves slower so SLAM can keep up

echo "=========================================="
echo "TurtleBot3 SLAM - SLOW MODE (Best Maps)"
echo "=========================================="
echo ""
echo "⚠️  Robot will move at 50% speed for better SLAM"
echo "    This produces the cleanest, most accurate maps"
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
echo "🗺️  Starting Cartographer SLAM (Ultra-Aggressive Mode)..."
echo ""

# Launch Cartographer SLAM with ultra-aggressive configuration
ros2 launch turtlebot3_cartographer cartographer.launch.py \
    use_sim_time:=True \
    configuration_basename:=turtlebot3_lds_2d_ultra.lua &
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
echo "🐌 Starting autonomous drive node (SLOW MODE for SLAM)..."
ros2 run turtlebot3_gazebo turtlebot3_drive --ros-args -p max_linear_speed:=0.08 -p max_angular_speed:=0.4 &
DRIVE_PID=$!

sleep 2

echo ""
echo "📺 Launching RViz2 with SLAM configuration..."
rviz2 -d slam_config.rviz &
RVIZ_PID=$!

sleep 2

echo ""
echo "✅ SLAM System Launched Successfully (SLOW MODE)!"
echo ""
echo "📊 What's Running:"
echo "   - Cartographer SLAM (ultra-aggressive mode)"
echo "   - Path Visualizer (tracking trajectory)"
echo "   - TurtleBot3 Drive (50% SPEED for better SLAM)"
echo ""
echo "🐌 SLOW MODE Settings:"
echo "   - Linear speed: 0.08 m/s (was 0.15)"
echo "   - Angular speed: 0.4 rad/s (was 1.0)"
echo "   - Optimizes every 3 scans (very frequent)"
echo "   - Loop closure: 0.40 threshold (extremely aggressive)"
echo ""
echo "💡 Why Slow?"
echo "   SLAM needs time to process scans and match features."
echo "   Slower movement = better scan matching = less drift!"
echo ""
echo "💾 To Save the Map:"
echo "   Run in a new terminal:"
echo "   ./save_map.sh"
echo ""
echo "Press Ctrl+C to stop all nodes"
echo ""

# Wait for user interrupt
wait
