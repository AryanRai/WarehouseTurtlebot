#!/bin/bash
# Autonomous SLAM System Runner
# Runs the complete autonomous SLAM system with frontier exploration

echo "🤖 Starting Autonomous SLAM System"
echo "=================================="
echo ""

# Set TurtleBot3 model environment variable
export TURTLEBOT3_MODEL=burger

cd "$(dirname "$0")/../turtlebot3_ws"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built! Please run './scripts/build_project.sh' first."
    exit 1
fi

source install/setup.bash

# Check if Gazebo is running
if ! pgrep -f "gz sim" > /dev/null; then
    echo "❌ Gazebo is not running!"
    echo "Please start Gazebo first with:"
    echo "   ./launch_mgen.sh"
    exit 1
fi

echo "✅ Gazebo is running"
echo ""

# Function to cleanup processes on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down Autonomous SLAM System..."
    
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
    
    if [ ! -z "$SLAM_PID" ] && ps -p $SLAM_PID > /dev/null 2>&1; then
        echo "   Stopping Autonomous SLAM Controller..."
        kill $SLAM_PID 2>/dev/null
    fi
    
    if [ ! -z "$SPAWN_PID" ] && ps -p $SPAWN_PID > /dev/null 2>&1; then
        echo "   Stopping TurtleBot3 spawn..."
        kill $SPAWN_PID 2>/dev/null
    fi
    
    echo "✅ Autonomous SLAM system stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo "🚀 Starting Autonomous SLAM Components..."
echo ""

echo "1️⃣ Starting robot_state_publisher..."
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True &
RSP_PID=$!
sleep 2

echo "2️⃣ Spawning TurtleBot3 at origin (0, 0) - autonomous mode..."
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0 &
SPAWN_PID=$!
sleep 4

# Kill the wall-following drive node (we don't need it for autonomous SLAM)
echo "🔧 Stopping wall-following node..."
pkill -f turtlebot3_drive_node
sleep 1

echo "✅ Robot spawned without wall following - ready for autonomous SLAM control"

echo "3️⃣ Starting Cartographer SLAM (output redirected to /tmp/cartographer.log)..."
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True > /tmp/cartographer.log 2>&1 &
CARTOGRAPHER_PID=$!

echo "⏳ Waiting for Cartographer to initialize..."
sleep 5

# Check if Cartographer started successfully
if ! ps -p $CARTOGRAPHER_PID > /dev/null 2>&1; then
    echo "❌ Cartographer failed to start!"
    cleanup
    exit 1
fi

echo "4️⃣ Launching RViz2 for visualization (output redirected to /tmp/rviz.log)..."
rviz2 > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 3

echo "5️⃣ Starting Autonomous SLAM Controller..."
echo "⏳ Waiting for SLAM to be ready..."
sleep 3

# Check if map topic is available before starting autonomous controller
echo "🔍 Checking for /map topic..."
timeout 10s bash -c 'until ros2 topic list | grep -q "^/map$"; do sleep 1; done' || {
    echo "⚠️  Map topic not available yet, but starting controller anyway..."
}

ros2 run warehouse_robot_system autonomous_slam_main &
SLAM_PID=$!
sleep 3

echo ""
echo "✅ Autonomous SLAM System Started!"
echo ""
echo "📊 Running Components:"
echo "   🔧 robot_state_publisher (PID: $RSP_PID)"
echo "   🤖 TurtleBot3 at origin (PID: $SPAWN_PID)"
echo "   🗺️  Cartographer SLAM (PID: $CARTOGRAPHER_PID)"
echo "   🖥️  RViz2 (PID: $RVIZ_PID)"
echo "   🧠 Autonomous SLAM Controller (PID: $SLAM_PID)"
echo ""
echo "🎯 System Behavior:"
echo "   • Robot will automatically explore the environment"
echo "   • Uses frontier detection to find unexplored areas"
echo "   • Plans optimal paths using A* algorithm"
echo "   • Returns to origin (0,0) when mapping is complete"
echo "   • Transitions to operational mode for warehouse tasks"
echo ""
echo "🖥️  RViz2 Setup:"
echo "   1. RViz2 should have opened automatically"
echo "   2. Add these displays if not visible:"
echo "      - Map (topic: /map) - shows the SLAM map"
echo "      - LaserScan (topic: /scan) - shows lidar data"
echo "      - Path (topic: /slam/planned_path) - shows planned paths"
echo "      - PoseStamped (topic: /slam/current_goal) - shows current goal"
echo "   3. Set Fixed Frame to: 'map'"
echo ""
echo "📈 Monitoring:"
echo "   • Watch RViz to see autonomous exploration"
echo "   • Robot will move to frontiers automatically"
echo "   • SLAM builds map as robot explores"
echo "   • System logs show state transitions and progress"
echo "   • Cartographer logs: tail -f /tmp/cartographer.log"
echo "   • RViz logs: tail -f /tmp/rviz.log"
echo ""
echo "🔄 State Machine:"
echo "   INITIALIZING → MAPPING → RETURNING_HOME → OPERATIONAL"
echo ""
echo "💡 The robot is now fully autonomous!"
echo "   No manual control needed - it will explore and map automatically."
echo ""
echo "Press Ctrl+C to stop all components"

# Wait for user interrupt
while true; do
    # Check if any critical process died
    if ! ps -p $SLAM_PID > /dev/null 2>&1; then
        echo "❌ Autonomous SLAM Controller process died!"
        break
    fi
    
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