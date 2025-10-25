#!/bin/bash
# Emergency Stop - Immediately stop TurtleBot3 movement
# Publishes zero velocity to /cmd_vel

# Set ROS Domain ID
export ROS_DOMAIN_ID=29
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "🛑 EMERGENCY STOP"
echo "=================="
echo "   Sending zero velocity to TurtleBot3..."
echo ""

# Publish zero velocity 5 times to ensure it's received
for i in {1..5}; do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null
    sleep 0.1
done

echo "✅ Zero velocity commands sent"
echo ""
echo "💡 The robot should stop immediately"
echo "   If it continues moving, check:"
echo "   - ROS_DOMAIN_ID is correct (currently: $ROS_DOMAIN_ID)"
echo "   - /cmd_vel topic exists: ros2 topic list | grep cmd_vel"
echo "   - Hardware bringup is running on TurtleBot"
