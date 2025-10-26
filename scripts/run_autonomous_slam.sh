#!/bin/bash
# Autonomous SLAM System Runner
# Runs the complete autonomous SLAM system with frontier exploration
# Supports both Gazebo simulation and physical TurtleBot3

# Set ROS Domain ID
export ROS_DOMAIN_ID=29
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "🤖 Starting Autonomous SLAM System"
echo "=================================="
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""

cd "$(dirname "$0")/../turtlebot3_ws"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built! Please run './scripts/build_project.sh' first."
    exit 1
fi

source install/setup.bash

# Check if Gazebo is running
USE_PHYSICAL_ROBOT=false
if ! pgrep -f "gz sim" > /dev/null; then
    echo "⚠️  Gazebo is not running!"
    echo ""
    echo "Would you like to run on physical TurtleBot3 instead? (y/n)"
    read -r response
    
    if [[ "$response" =~ ^[Yy]$ ]]; then
        USE_PHYSICAL_ROBOT=true
        echo ""
        echo "🤖 Switching to Physical TurtleBot3 Mode"
        echo "========================================"
        echo ""
        echo "Prerequisites:"
        echo "1. TurtleBot3 must be powered on and connected"
        echo "2. Hardware bringup must be running on TurtleBot"
        echo ""
        echo "💡 To start hardware bringup, run:"
        echo "   ./scripts/turtlebot_bringup.sh start"
        echo ""
        echo "Press Enter to continue or Ctrl+C to cancel..."
        read -r
    else
        echo ""
        echo "❌ Please start Gazebo first with:"
        echo "   ./launch_mgen.sh"
        exit 1
    fi
else
    echo "✅ Gazebo is running - using simulation mode"
    echo ""
fi

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
    
    if [ ! -z "$BATTERY_PID" ] && ps -p $BATTERY_PID > /dev/null 2>&1; then
        echo "   Stopping Battery Monitor..."
        kill $BATTERY_PID 2>/dev/null
    fi
    
    if [ ! -z "$BATTERY_DISPLAY_PID" ] && ps -p $BATTERY_DISPLAY_PID > /dev/null 2>&1; then
        echo "   Stopping Battery Display..."
        kill $BATTERY_DISPLAY_PID 2>/dev/null
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

# Different startup for physical robot vs simulation
if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "🤖 Physical TurtleBot3 Mode"
    echo "==========================="
    echo ""
    echo "Skipping robot spawn (using physical robot)"
    echo "Make sure hardware bringup is running on TurtleBot!"
    echo ""
    
    # Set use_sim_time to false for physical robot
    USE_SIM_TIME="False"
    RSP_PID=""
    SPAWN_PID=""
else
    echo "🎮 Simulation Mode"
    echo "=================="
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
    
    # Set use_sim_time to true for simulation
    USE_SIM_TIME="True"
fi

echo "3️⃣ Starting Cartographer SLAM (output redirected to /tmp/cartographer.log)..."
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=$USE_SIM_TIME > /tmp/cartographer.log 2>&1 &
CARTOGRAPHER_PID=$!

echo "⏳ Waiting for Cartographer to initialize..."
sleep 5

# Check if Cartographer started successfully
if ! ps -p $CARTOGRAPHER_PID > /dev/null 2>&1; then
    echo "❌ Cartographer failed to start!"
    cleanup
    exit 1
fi

echo "4️⃣ Starting Battery Monitoring System..."

# Get absolute path to config file
BATTERY_CONFIG="$(pwd)/src/mtrx3760_battery/config/battery_params.yaml"

if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "   Using real battery data from /battery_state"
    ros2 run mtrx3760_battery battery_monitor_node --ros-args --params-file "$BATTERY_CONFIG" -p use_simulation:=false > /tmp/battery_monitor.log 2>&1 &
    BATTERY_PID=$!
else
    echo "   Using battery simulator for Gazebo"
    ros2 run mtrx3760_battery battery_simulator_node --ros-args --params-file "$BATTERY_CONFIG" > /tmp/battery_simulator.log 2>&1 &
    BATTERY_PID=$!
fi

echo "   Starting battery terminal display..."
ros2 run mtrx3760_battery battery_terminal_display --ros-args --params-file "$BATTERY_CONFIG" > /tmp/battery_display.log 2>&1 &
BATTERY_DISPLAY_PID=$!
sleep 2

echo "5️⃣ Launching RViz2 for visualization (output redirected to /tmp/rviz.log)..."
rviz2 > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
sleep 3

echo "6️⃣ Starting Autonomous SLAM Controller..."
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
if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "   🤖 Physical TurtleBot3 (hardware bringup on robot)"
else
    echo "   🔧 robot_state_publisher (PID: $RSP_PID)"
    echo "   🤖 TurtleBot3 in Gazebo (PID: $SPAWN_PID)"
fi
echo "   🗺️  Cartographer SLAM (PID: $CARTOGRAPHER_PID)"
echo "   🔋 Battery Monitor (PID: $BATTERY_PID)"
echo "   📊 Battery Display (PID: $BATTERY_DISPLAY_PID)"
echo "   🖥️  RViz2 (PID: $RVIZ_PID)"
echo "   🧠 Autonomous SLAM Controller (PID: $SLAM_PID)"
echo ""
echo "🎯 Mode: $([ "$USE_PHYSICAL_ROBOT" = true ] && echo "Physical Robot" || echo "Simulation")"
echo "   use_sim_time: $USE_SIM_TIME"
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
echo "   • Battery status displayed in terminal"
echo "   • System logs show state transitions and progress"
echo "   • Cartographer logs: tail -f /tmp/cartographer.log"
echo "   • Battery logs: tail -f /tmp/battery_display.log"
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
    
    if ! ps -p $BATTERY_PID > /dev/null 2>&1; then
        echo "⚠️  Battery Monitor process died (non-critical)"
    fi
    
    # Only check RSP if in simulation mode
    if [ "$USE_PHYSICAL_ROBOT" = false ] && [ ! -z "$RSP_PID" ]; then
        if ! ps -p $RSP_PID > /dev/null 2>&1; then
            echo "❌ robot_state_publisher process died!"
            break
        fi
    fi
    
    sleep 1
done

cleanup