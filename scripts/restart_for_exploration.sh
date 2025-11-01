#!/bin/bash
# Quick restart script for switching to exploration mode
# This ensures clean TF transforms by restarting Gazebo

echo "🔄 Restarting System for Exploration Mode"
echo "=========================================="
echo ""
echo "This script will:"
echo "  1. Stop all running ROS nodes"
echo "  2. Stop Gazebo simulation"
echo "  3. Restart Gazebo with warehouse"
echo "  4. Start exploration mode"
echo ""
echo "⚠️  Make sure you've saved any important data!"
echo ""
echo -n "Continue? (yes/no): "
read -r confirm

if [[ "$confirm" != "yes" ]]; then
    echo "Cancelled."
    exit 0
fi

echo ""
echo "🛑 Stopping all ROS nodes..."
pkill -f "ros2 run warehouse_robot_system"
pkill -f "ros2 launch slam_toolbox"
pkill -f "ros2 run rviz2"
sleep 2

echo "🛑 Stopping Gazebo..."
pkill -f "gz sim"
pkill -f "gzserver"
pkill -f "gzclient"
pkill -f "ros_gz_bridge"
sleep 3

echo ""
echo "✅ System stopped. Starting fresh..."
echo ""
echo "📋 Next steps:"
echo "   1. Run: ./launch_warehouse.sh"
echo "   2. Wait for Gazebo to fully load"
echo "   3. Run: ./scripts/run_autonomous_slam.sh"
echo "   4. Select option 1 (Exploration Mode)"
echo ""
echo "💡 This ensures clean TF transforms!"
echo ""
