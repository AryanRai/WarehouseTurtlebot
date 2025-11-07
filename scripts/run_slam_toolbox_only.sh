#!/bin/bash
# Run SLAM Toolbox mapping without autonomous controller
# Use this for manual control or testing
#
# Usage:
#   ./scripts/run_slam_toolbox_only.sh

export ROS_DOMAIN_ID=29
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo " Starting SLAM Toolbox Mapping (Manual Control Mode)"
echo "======================================================"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$(dirname "$0")/../turtlebot3_ws"

if [ ! -d "install" ]; then
    echo " Workspace not built! Please run './scripts/build_project.sh' first."
    exit 1
fi

source install/setup.bash

# Check if Gazebo is running
if ! pgrep -f "gz sim" > /dev/null; then
    echo " Gazebo is not running!"
    echo "   Please start Gazebo first with: ./launch_mgen.sh"
    exit 1
fi

echo " Gazebo is running"
echo ""

# Function to cleanup processes on exit
cleanup() {
    echo ""
    echo " Shutting down SLAM Toolbox..."
    
    if [ ! -z "$RSP_PID" ] && ps -p $RSP_PID > /dev/null 2>&1; then
        echo "   Stopping robot_state_publisher..."
        kill $RSP_PID 2>/dev/null
    fi
    
    if [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
        echo "   Stopping SLAM Toolbox..."
        kill $SLAM_TOOLBOX_PID 2>/dev/null
    fi
    
    if [ ! -z "$RVIZ_PID" ] && ps -p $RVIZ_PID > /dev/null 2>&1; then
        echo "   Stopping RViz..."
        kill $RVIZ_PID 2>/dev/null
    fi
    
    if [ ! -z "$SPAWN_PID" ] && ps -p $SPAWN_PID > /dev/null 2>&1; then
        echo "   Stopping TurtleBot3 spawn..."
        kill $SPAWN_PID 2>/dev/null
    fi
    
    echo " SLAM Toolbox stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

echo " Starting Components..."
echo ""

echo "1️⃣ Starting robot_state_publisher..."
ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true &
RSP_PID=$!
sleep 2

echo "2️⃣ Spawning TurtleBot3 at origin (0, 0)..."
ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0 &
SPAWN_PID=$!
sleep 4

echo " Stopping wall-following node..."
pkill -f turtlebot3_drive_node
sleep 1

echo "3️⃣ Starting SLAM Toolbox..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true > /tmp/slam_toolbox.log 2>&1 &
SLAM_TOOLBOX_PID=$!
sleep 5

if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
    echo " SLAM Toolbox failed to start!"
    echo "Check logs: tail -f /tmp/slam_toolbox.log"
    cleanup
    exit 1
fi

echo "4️⃣ Launching RViz2..."
RVIZ_CONFIG="$(pwd)/install/warehouse_robot_system/share/warehouse_robot_system/config/slam_toolbox.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$(pwd)/src/turtlebot3_simulations/warehouse_robot_system/config/slam_toolbox.rviz"
fi

if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz.log 2>&1 &
else
    rviz2 > /tmp/rviz.log 2>&1 &
fi
RVIZ_PID=$!
sleep 3

echo ""
echo " SLAM Toolbox Mapping Started!"
echo ""
echo " Running Components:"
echo "    robot_state_publisher (PID: $RSP_PID)"
echo "    TurtleBot3 in Gazebo (PID: $SPAWN_PID)"
echo "   ️  SLAM Toolbox (PID: $SLAM_TOOLBOX_PID)"
echo "   ️  RViz2 (PID: $RVIZ_PID)"
echo ""
echo " Control the Robot:"
echo "   Open another terminal and run:"
echo "   ./scripts/run_teleop.sh"
echo ""
echo " Save the Map:"
echo "   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \\"
echo "     \"{name: {data: '/tmp/warehouse_map'}}\""
echo ""
echo " Monitoring:"
echo "   • SLAM Toolbox logs: tail -f /tmp/slam_toolbox.log"
echo "   • RViz logs: tail -f /tmp/rviz.log"
echo "   • Map topic: ros2 topic echo /map --once"
echo ""
echo "Press Ctrl+C to stop all components"

# Wait for user interrupt
while true; do
    if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
        echo " SLAM Toolbox process died!"
        break
    fi
    
    if ! ps -p $RSP_PID > /dev/null 2>&1; then
        echo " robot_state_publisher process died!"
        break
    fi
    
    sleep 1
done

cleanup
