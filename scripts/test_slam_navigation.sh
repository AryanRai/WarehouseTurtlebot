#!/bin/bash
# Test script for SLAM navigation fixes
# This script helps monitor the robot's behavior to verify the circling fix

echo "=== SLAM Navigation Test Monitor ==="
echo ""
echo "This script will monitor key topics to help diagnose navigation issues."
echo "Watch for:"
echo "  1. Velocity commands (should be smooth, not erratic)"
echo "  2. Path updates (should change as robot explores)"
echo "  3. Goal positions (should move to new frontiers, not circle)"
echo ""
echo "Press Ctrl+C to stop monitoring"
echo ""

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source ~/MTRX3760_Project_2/turtlebot3_ws/install/setup.bash

# Function to monitor a topic briefly
monitor_topic() {
    local topic=$1
    local msg_type=$2
    local duration=$3
    
    echo "--- Monitoring $topic for ${duration}s ---"
    timeout ${duration}s ros2 topic echo $topic --once 2>/dev/null || echo "  (no data)"
    echo ""
}

# Main monitoring loop
while true; do
    clear
    echo "=== SLAM Navigation Monitor ($(date +%H:%M:%S)) ==="
    echo ""
    
    # Check current velocity command
    echo "Current Velocity Command:"
    monitor_topic "/cmd_vel" "geometry_msgs/msg/TwistStamped" 1
    
    # Check current goal
    echo "Current Goal:"
    monitor_topic "/slam/current_goal" "geometry_msgs/msg/PoseStamped" 1
    
    # Check path
    echo "Current Path (number of poses):"
    ros2 topic echo /slam/planned_path --once 2>/dev/null | grep -c "pose:" || echo "  (no path)"
    echo ""
    
    # Check robot pose
    echo "Robot Pose:"
    ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | head -4 || echo "  (no data)"
    echo ""
    
    # Wait before next update
    sleep 2
done
