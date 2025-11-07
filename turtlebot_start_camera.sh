#!/bin/bash

# ============================================================================
# TurtleBot Camera Detection Startup Script
# Run this ON THE TURTLEBOT after building the camera package
# ============================================================================

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   TurtleBot Camera Detection System                   ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Source ROS2
echo " Sourcing ROS2 environment..."
source /opt/ros/jazzy/setup.bash

# Check if workspace exists
if [ ! -d "$HOME/camera_ws" ]; then
    echo -e "${RED} Camera workspace not found at ~/camera_ws${NC}"
    echo ""
    echo "Please build the camera package first:"
    echo "  cd ~/camera_ws"
    echo "  colcon build --packages-select turtlebot_camera"
    exit 1
fi

# Source workspace
if [ -f "$HOME/camera_ws/install/setup.bash" ]; then
    echo " Sourcing camera workspace..."
    source "$HOME/camera_ws/install/setup.bash"
else
    echo -e "${RED} Camera workspace not built!${NC}"
    echo ""
    echo "Please build the camera package first:"
    echo "  cd ~/camera_ws"
    echo "  colcon build --packages-select turtlebot_camera"
    exit 1
fi

# Set ROS Domain ID (match your laptop and robot hardware)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-29}

echo ""
echo -e "${GREEN} Environment ready${NC}"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo " Stopping camera detection..."
    if [ ! -z "$APRILTAG_PID" ]; then
        kill $APRILTAG_PID 2>/dev/null
    fi
    if [ ! -z "$COLOR_PID" ]; then
        kill $COLOR_PID 2>/dev/null
    fi
    echo " Stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start AprilTag detector
echo "️  Starting AprilTag detector (headless mode)..."
ros2 run turtlebot_camera apriltag_detector_node \
    --ros-args \
    -p show_visualization:=false \
    -p print_detections:=true \
    -p enable_temporal_filtering:=false &

APRILTAG_PID=$!
sleep 2

if ps -p $APRILTAG_PID > /dev/null; then
    echo -e "${GREEN} AprilTag detector started (PID: $APRILTAG_PID)${NC}"
else
    echo -e "${RED} Failed to start AprilTag detector${NC}"
    echo ""
    echo "Check logs:"
    echo "  ros2 run turtlebot_camera apriltag_detector_node"
    exit 1
fi

# Start Color detector
echo ""
echo " Starting Color detector..."
ros2 run turtlebot_camera colour_detector_node \
    --ros-args \
    -p calibration_mode:=false &

COLOR_PID=$!
sleep 2

if ps -p $COLOR_PID > /dev/null; then
    echo -e "${GREEN} Color detector started (PID: $COLOR_PID)${NC}"
else
    echo -e "${YELLOW}️  Color detector failed to start (optional)${NC}"
fi

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║   Camera Detection Running on TurtleBot               ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo " Status:"
echo "   • AprilTag detector: Running (PID: $APRILTAG_PID)"
if ps -p $COLOR_PID > /dev/null 2>/dev/null; then
    echo "   • Color detector: Running (PID: $COLOR_PID)"
else
    echo "   • Color detector: Not running"
fi
echo ""
echo " Publishing to:"
echo "   • /apriltag_detections"
echo "   • /color_detections"
echo ""
echo " Benefits:"
echo "   • 300x less network traffic"
echo "   • No camera stream over WiFi"
echo "   • Only detection results sent"
echo "   • More reliable TF2"
echo ""
echo " Press Ctrl+C to stop"
echo ""

# Wait for processes
wait
