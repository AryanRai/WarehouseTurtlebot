#!/bin/bash
# ROS Link TurtleBot - Set environment to connect to TurtleBot3
# This script kills local ROS processes and configures environment for physical TurtleBot3

echo " Connecting to Physical TurtleBot3"
echo "====================================="
echo ""

# Step 1: Kill all local ROS processes
echo "1️⃣ Stopping local ROS processes..."
SCRIPT_DIR="$(dirname "$0")"
bash "$SCRIPT_DIR/kill_all_ros.sh"
sleep 2

# Step 2: Set RMW implementation to FastRTPS
echo ""
echo "2️⃣ Setting RMW implementation..."
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "    RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# Step 3: Set ROS Domain ID
echo ""
echo "3️⃣ Setting ROS Domain ID..."
export ROS_DOMAIN_ID=29
export TURTLEBOT3_MODEL=burger
echo "    ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "    TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"

# Step 4: Restart ROS2 daemon with new settings
echo ""
echo "4️⃣ Restarting ROS2 daemon with new domain..."
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null
sleep 2

# Step 5: List available topics
echo ""
echo "5️⃣ Checking connection to TurtleBot3..."
echo "   Available topics:"
if ros2 topic list 2>/dev/null; then
    echo ""
    echo " Connected to TurtleBot3 on domain 29!"
else
    echo ""
    echo "️  Could not list topics (TurtleBot may not be running yet)"
    echo "   Try running: ros2 topic list"
fi
echo ""
echo " You can now run commands like:"
echo "   $ ros2 topic echo /scan"
echo "   $ ros2 topic echo /odom"
echo "   $ ros2 run turtlebot3_teleop teleop_keyboard"
echo "   $ ros2 launch turtlebot3_cartographer cartographer.launch.py"
echo ""
echo " Note: Environment is set for this terminal session only."
