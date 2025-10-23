#!/bin/bash
# SLAM Simulation Runner for Warehouse Robot System
# Based on working reference scripts from previous assignment

echo "=========================================="
echo "SLAM Simulation for Warehouse Robot System"
echo "=========================================="
echo ""

# Set up environment
export TURTLEBOT3_MODEL=burger
cd "$(dirname "$0")/../turtlebot3_ws"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built! Please run 'colcon build' first."
    exit 1
fi

source install/setup.bash

# Add TurtleBot3 models to Gazebo resource path
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models

# Check if Gazebo is already running
if ! pgrep -f "gz sim" > /dev/null; then
    echo "❌ Gazebo is not running!"
    echo "Please run ./launch_mgen.sh first to start the maze simulation"
    echo "Or run ./scripts/run_full_slam_demo.sh for a complete demo"
    exit 1
fi

echo "✅ Gazebo is already running"
echo ""
echo "🚀 Starting SLAM Simulation Components..."
echo ""

# Function to cleanup processes on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down SLAM simulation..."
    
    # Kill background processes
    if [ ! -z "$RSP_PID" ] && ps -p $RSP_PID > /dev/null 2>&1; then
        echo "   Stopping robot_state_publisher..."
        kill $RSP_PID 2>/dev/null
    fi
    
    if [ ! -z "$CARTOGRAPHER_PID" ] && ps -p $CARTOGRAPHER_PID > /dev/null 2>&1; then
        echo "   Stopping Cartographer SLAM..."
        kill $CARTOGRAPHER_PID 2>/dev/null
    fi
    
    if [ ! -z "$RVIZ_PID" ] && ps -p $RVIZ_PID > /dev/null 2>&1; then
        echo "   Stopping RViz..."
        kill $RVIZ_PID 2>/dev/null
    fi
    
    if [ ! -z "$WAREHOUSE_PID" ] && ps -p $WAREHOUSE_PID > /dev/null 2>&1; then
        echo "   Stopping Warehouse Robot System..."
        kill $WAREHOUSE_PID 2>/dev/null
    fi
    
    if [ ! -z "$SPAWN_PID" ] && ps -p $SPAWN_PID > /dev/null 2>&1; then
        echo "   Stopping TurtleBot3 spawn..."
        kill $SPAWN_PID 2>/dev/null
    fi
    
    echo "✅ SLAM simulation stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo "1️⃣ Starting robot_state_publisher (TF publisher)..."
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True &
RSP_PID=$!
sleep 2

echo "2️⃣ Spawning TurtleBot3 at start position..."
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=1.0 y_pose:=-1.0 &
SPAWN_PID=$!
sleep 4

echo "3️⃣ Starting Cartographer SLAM..."
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &
CARTOGRAPHER_PID=$!

echo "⏳ Waiting for Cartographer to initialize..."
sleep 5

# Check if Cartographer started successfully
if ! ps -p $CARTOGRAPHER_PID > /dev/null 2>&1; then
    echo "❌ Cartographer failed to start!"
    cleanup
    exit 1
fi

echo "4️⃣ Launching RViz2 with SLAM configuration..."
rviz2 &
RVIZ_PID=$!
sleep 3

echo "5️⃣ Starting Warehouse Robot System..."
ros2 run warehouse_robot_system warehouse_robot_main &
WAREHOUSE_PID=$!
sleep 2

echo ""
echo "✅ SLAM Simulation Components Started!"
echo ""
echo "📊 Running Components:"
echo "   🔧 robot_state_publisher (PID: $RSP_PID)"
echo "   🤖 TurtleBot3 (PID: $SPAWN_PID)"
echo "   🗺️  Cartographer SLAM (PID: $CARTOGRAPHER_PID)"
echo "   🖥️  RViz2 (PID: $RVIZ_PID)"
echo "   🏭 Warehouse Robot System (PID: $WAREHOUSE_PID)"
echo ""
echo "🖥️  RViz2 Setup:"
echo "   1. RViz2 should have opened automatically"
echo "   2. Add these displays if not visible:"
echo "      - Map (topic: /map) - shows the SLAM map"
echo "      - LaserScan (topic: /scan) - shows lidar data"
echo "      - RobotModel - shows the robot"
echo "   3. Set Fixed Frame to: 'map'"
echo ""
echo "🎮 Controls:"
echo "   • Use keyboard teleop in another terminal:"
echo "     ros2 run turtlebot3_teleop teleop_keyboard"
echo "   • Or run autonomous navigation with:"
echo "     ros2 run warehouse_robot_system slam_test"
echo ""
echo "📈 Monitoring:"
echo "   • RViz should show the robot and building map"
echo "   • Warehouse system will show robot status updates"
echo "   • SLAM will build the map as robot moves"
echo ""
echo "💡 Tips:"
echo "   • Move the robot around to build a complete map"
echo "   • Watch the warehouse robot system output for polymorphic behavior"
echo "   • Use 'ros2 topic list' to see available topics"
echo ""
echo "Press Ctrl+C to stop all components"

# Wait for user interrupt
while true; do
    # Check if any critical process died
    if ! ps -p $CARTOGRAPHER_PID > /dev/null 2>&1; then
        echo "❌ Cartographer SLAM process died!"
        break
    fi
    
    if ! ps -p $RSP_PID > /dev/null 2>&1; then
        echo "❌ robot_state_publisher process died!"
        break
    fi
    
    sleep 1
done

cleanup