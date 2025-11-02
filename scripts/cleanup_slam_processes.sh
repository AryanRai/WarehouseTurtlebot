#!/bin/bash
# Cleanup script for SLAM Toolbox and related processes
# Use this if you're experiencing TF errors or stale processes

echo "🧹 Cleaning up SLAM and ROS processes..."
echo ""

# Kill SLAM Toolbox
SLAM_PIDS=$(pgrep -f "slam_toolbox" | tr '\n' ' ')
if [ ! -z "$SLAM_PIDS" ]; then
    echo "   Killing SLAM Toolbox processes: $SLAM_PIDS"
    for pid in $SLAM_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill autonomous_slam_node
SLAM_NODE_PIDS=$(pgrep -f "autonomous_slam_node" | tr '\n' ' ')
if [ ! -z "$SLAM_NODE_PIDS" ]; then
    echo "   Killing autonomous_slam_node processes: $SLAM_NODE_PIDS"
    for pid in $SLAM_NODE_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill robot_state_publisher
RSP_PIDS=$(pgrep -f "robot_state_publisher" | tr '\n' ' ')
if [ ! -z "$RSP_PIDS" ]; then
    echo "   Killing robot_state_publisher processes: $RSP_PIDS"
    for pid in $RSP_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill rosbridge
ROSBRIDGE_PIDS=$(pgrep -f "rosbridge" | tr '\n' ' ')
if [ ! -z "$ROSBRIDGE_PIDS" ]; then
    echo "   Killing rosbridge processes: $ROSBRIDGE_PIDS"
    for pid in $ROSBRIDGE_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill RViz
RVIZ_PIDS=$(pgrep -f "rviz2" | tr '\n' ' ')
if [ ! -z "$RVIZ_PIDS" ]; then
    echo "   Killing RViz processes: $RVIZ_PIDS"
    for pid in $RVIZ_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill delivery robot
DELIVERY_PIDS=$(pgrep -f "delivery_robot_node" | tr '\n' ' ')
if [ ! -z "$DELIVERY_PIDS" ]; then
    echo "   Killing delivery_robot_node processes: $DELIVERY_PIDS"
    for pid in $DELIVERY_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill inspection robot
INSPECTION_PIDS=$(pgrep -f "inspection_robot_node" | tr '\n' ' ')
if [ ! -z "$INSPECTION_PIDS" ]; then
    echo "   Killing inspection_robot_node processes: $INSPECTION_PIDS"
    for pid in $INSPECTION_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill AprilTag detector
APRILTAG_PIDS=$(pgrep -f "apriltag_detector_node" | tr '\n' ' ')
if [ ! -z "$APRILTAG_PIDS" ]; then
    echo "   Killing apriltag_detector_node processes: $APRILTAG_PIDS"
    for pid in $APRILTAG_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill camera node
CAMERA_PIDS=$(pgrep -f "camera_node" | tr '\n' ' ')
if [ ! -z "$CAMERA_PIDS" ]; then
    echo "   Killing camera_node processes: $CAMERA_PIDS"
    for pid in $CAMERA_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Kill color detector
COLOR_PIDS=$(pgrep -f "colour_detector_node" | tr '\n' ' ')
if [ ! -z "$COLOR_PIDS" ]; then
    echo "   Killing colour_detector_node processes: $COLOR_PIDS"
    for pid in $COLOR_PIDS; do
        kill -9 $pid 2>/dev/null
    done
fi

# Wait for processes to die
sleep 2

echo ""
echo "✅ Cleanup complete!"
echo ""
echo "💡 TF transforms should now be clear"
echo "   You can now start the system fresh"
echo ""
