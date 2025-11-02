#!/bin/bash
# Emergency Stop - Immediately stop TurtleBot3 movement
# Publishes zero velocity to /cmd_vel and kills autonomous controller

# Set ROS Domain ID
export ROS_DOMAIN_ID=29
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "🛑 EMERGENCY STOP"
echo "=================="
echo ""

# Check if ROS2 is running
if ! ros2 topic list &>/dev/null; then
    echo "⚠️  Warning: ROS2 nodes may not be running"
fi

# Step 1: Kill autonomous controller
echo "1️⃣ Stopping autonomous SLAM controller..."
pkill -9 -f autonomous_slam
pkill -9 -f run_autonomous_slam
sleep 0.5

# Step 2: Publish zero velocity multiple times with timeout
echo "2️⃣ Sending zero velocity commands..."
for i in {1..5}; do
    timeout 1s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null &
done
sleep 1

# Step 3: Start continuous zero velocity publisher for 2 seconds
echo "3️⃣ Holding zero velocity..."
timeout 2s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10 2>/dev/null
sleep 0.5

echo ""
echo "✅ ROBOT STOPPED"
echo ""
echo "💡 What was done:"
echo "   - Killed autonomous SLAM controller"
echo "   - Sent multiple zero velocity commands"
echo "   - Held zero velocity for 2 seconds"
echo ""
echo "🔍 To verify robot stopped:"
echo "   ros2 topic echo /cmd_vel --once"
echo "   ros2 topic echo /odom --once  # Check velocities are zero"
