#!/bin/bash
# Force Kill All ROS - Quick and aggressive cleanup

echo " FORCE KILLING ALL ROS PROCESSES"
echo "=================================="
echo ""

# Kill everything ROS-related immediately
echo " Force killing all ROS2, Gazebo, and related processes..."

# Kill by process name patterns (force kill)
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "gazebo" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "rviz" 2>/dev/null || true
pkill -9 -f "cartographer" 2>/dev/null || true
pkill -9 -f "turtlebot3" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "autonomous_slam" 2>/dev/null || true
pkill -9 -f "warehouse_robot" 2>/dev/null || true
pkill -9 -f "nav2" 2>/dev/null || true
pkill -9 -f "navigation2" 2>/dev/null || true
pkill -9 -f "slam_toolbox" 2>/dev/null || true
pkill -9 -f "map_server" 2>/dev/null || true
pkill -9 -f "amcl" 2>/dev/null || true

# Kill by exact command names
pkill -9 "gz" 2>/dev/null || true
pkill -9 "rviz2" 2>/dev/null || true
pkill -9 "rqt" 2>/dev/null || true

# Clean up ROS2 daemon
pkill -9 -f "_ros2_daemon" 2>/dev/null || true
rm -rf /tmp/ros2_daemon_* 2>/dev/null || true

# Clean shared memory
ipcs -m | grep $(whoami) | awk '{print $2}' | xargs -r ipcrm -m 2>/dev/null || true

echo " Force kill complete!"
echo ""
echo " Checking for survivors..."
survivors=$(pgrep -f "ros2\|gazebo\|gz\|rviz\|cartographer\|turtlebot3" 2>/dev/null || true)
if [ -z "$survivors" ]; then
    echo " All ROS processes terminated"
else
    echo "️  Some processes survived:"
    ps -p $survivors -o pid,cmd --no-headers 2>/dev/null || true
    echo ""
    echo " Nuclear option - killing by PID..."
    echo "$survivors" | xargs -r kill -9 2>/dev/null || true
fi

echo ""
echo " FORCE CLEANUP COMPLETE!"
echo "Ready to restart ROS2 services."