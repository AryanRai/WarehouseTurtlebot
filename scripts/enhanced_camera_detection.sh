#!/bin/bash

# ============================================================================
# TurtleBot Camera Calibration System - Enhanced with Independent AprilTag Detection
# Description: Complete calibration system with separate AprilTag and color detection
# Author(s): Dylan George
# Last Edited: 2025-11-01
# ============================================================================

# Colors and formatting
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to display header
show_header() {
    echo -e "${BLUE}🎨 TurtleBot Enhanced Camera Detection System${NC}"
    echo "========================================"
    echo ""
    echo -e "${GREEN}🔧 Prerequisites:${NC}"
    echo "  1. SSH into TurtleBot: ssh -X ubuntu@TURTLEBOT_IP"
    echo "  2. Run: ros2 launch turtlebot3_bringup robot.launch.py"
    echo "  3. Verify camera: ros2 topic hz /camera/image_raw"
    echo ""
    echo -e "${YELLOW}📖 How the Enhanced System Works:${NC}"
    echo "  • ${GREEN}Independent AprilTag Detection:${NC} Dedicated 16h5 tag detector with orientation"
    echo "  • ${BLUE}Color Detection:${NC} HSV-based damage classification around detected tags"
    echo "  • ${RED}GUI Visualization:${NC} Live camera feed with tag boxes and color regions"
    echo "  • ${YELLOW}Adaptive Sampling:${NC} Regions scale with tag distance for accurate detection"
    echo ""
}

# Function to check if processes are running
check_processes() {
    apriltag_pid=$(pgrep -f "apriltag_detector_node" || echo "")
    color_pid=$(pgrep -f "colour_detector_node" || echo "")
    
    if [[ -n "$apriltag_pid" ]]; then
        echo -e "   ${GREEN}✅ AprilTag detector running (PID: $apriltag_pid)${NC}"
    else
        echo -e "   ${RED}❌ AprilTag detector not running${NC}"
    fi
    
    if [[ -n "$color_pid" ]]; then
        echo -e "   ${GREEN}✅ Color detector running (PID: $color_pid)${NC}"
    else
        echo -e "   ${RED}❌ Color detector not running${NC}"
    fi
}

# Function to kill all detection processes
kill_detectors() {
    echo "🛑 Stopping all detection processes..."
    pkill -f "apriltag_detector_node" 2>/dev/null
    pkill -f "colour_detector_node" 2>/dev/null
    sleep 2
}

# Function to setup environment
setup_environment() {
    # Set up logging
    export ROS_LOG_DIR=/tmp/ros_logs
    mkdir -p $ROS_LOG_DIR
    export RCUTILS_LOGGING_USE_STDOUT=1
    export RCUTILS_COLORIZED_OUTPUT=1
    
    # Fix GUI display issues
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export DISPLAY=${DISPLAY:-:0}
    
    # Source ROS workspace
    cd ~/MTRX3760_Project_2/turtlebot3_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
}

show_header

while true; do
    echo -e "${BLUE}Choose detection mode:${NC}"
    echo "  1) 🏷️ AprilTag Detection Only (ID, position, orientation)"
    echo "  2) 🎨 Color Calibration Mode (HSV tuning with AprilTag detection)"
    echo "  3) 🔍 Full Detection System (AprilTag + Color detection)"
    echo "  4) 🚀 HEADLESS Mode - No GUI (Fast, for SSH without -X)"
    echo "  5) 📋 System Status & Process Management"
    echo "  6) 🛑 Stop All Detectors"
    echo "  7) 🚪 Exit"
    echo ""
    echo -n "Enter choice [1-7]: "
    read choice

    case $choice in
        1)
            echo ""
            echo -e "${GREEN}🏷️ Starting AprilTag Detection Only...${NC}"
            echo "======================================="
            echo ""
            echo "This will show:"
            echo "  📹 Live camera feed with 16h5 AprilTag detection"
            echo "  🟢 Green bounding boxes around detected tags"
            echo "  🏷️ Tag ID labels and orientation information"
            echo "  📊 Console output with position and orientation data"
            echo ""
            
            setup_environment
            kill_detectors
            
            echo "Starting AprilTag detector..."
            ros2 run warehouse_robot_system apriltag_detector_node \
                --ros-args \
                -p show_visualization:=true \
                -p print_detections:=true &
            
            echo ""
            echo -e "${YELLOW}🎮 Press Ctrl+C to stop detection${NC}"
            wait
            ;;
            
        2)
            echo ""
            echo -e "${BLUE}🎨 Starting Color Calibration Mode...${NC}"
            echo "====================================="
            echo ""
            echo "This will show:"
            echo "  📹 Live camera feed with color sampling regions"
            echo "  🏷️ AprilTag detection boxes (from detector node)"
            echo "  🟩🟦🟥 Adaptive sampling rectangles around tags"
            echo "  📊 Real-time HSV values and calibration guidance"
            echo ""
            echo "Controls:"
            echo "  's' = Save calibration to ~/hsv_calibration.yaml"
            echo "  'c' = Print HSV values and placement guidance"
            echo "  'q' = Quit"
            echo ""
            
            setup_environment
            kill_detectors
            
            echo "Starting AprilTag detector (background)..."
            ros2 run warehouse_robot_system apriltag_detector_node \
                --ros-args \
                -p show_visualization:=false \
                -p print_detections:=false &
            
            sleep 2
            
            echo "Starting color detector in calibration mode..."
            ros2 run warehouse_robot_system colour_detector_node \
                --ros-args -p calibration_mode:=true
            ;;
            
        3)
            echo ""
            echo -e "${RED}🔍 Starting Full Detection System...${NC}"
            echo "===================================="
            echo ""
            echo "This will run:"
            echo "  🏷️ AprilTag detector (ID, position, orientation)"
            echo "  🎨 Color detector (damage classification)"
            echo "  📺 Two visualization windows"
            echo "  📊 Console output from both detectors"
            echo ""
            
            setup_environment
            kill_detectors
            
            echo "Starting AprilTag detector..."
            ros2 run warehouse_robot_system apriltag_detector_node \
                --ros-args \
                -p show_visualization:=true \
                -p print_detections:=true &
            
            sleep 2
            
            echo "Starting color detector..."
            ros2 run warehouse_robot_system colour_detector_node \
                --ros-args -p calibration_mode:=false &
            
            echo ""
            echo -e "${YELLOW}🎮 Press Ctrl+C to stop both detectors${NC}"
            wait
            ;;
            
        4)
            echo ""
            echo -e "${GREEN}🚀 Starting HEADLESS Detection System...${NC}"
            echo "========================================="
            echo ""
            echo "This will run:"
            echo "  🏷️ AprilTag detector (NO GUI - console only)"
            echo "  🎨 Color detector (NO GUI - console only)"
            echo "  📊 Console output from both detectors"
            echo "  ⚡ FAST - No X11 overhead"
            echo ""
            echo -e "${YELLOW}💡 Perfect for SSH without -X flag!${NC}"
            echo ""
            
            setup_environment
            kill_detectors
            
            echo "Starting AprilTag detector (headless)..."
            ros2 run warehouse_robot_system apriltag_detector_node \
                --ros-args \
                -p show_visualization:=false \
                -p print_detections:=true &
            
            sleep 2
            
            echo "Starting color detector (headless)..."
            ros2 run warehouse_robot_system colour_detector_node \
                --ros-args -p calibration_mode:=false &
            
            echo ""
            echo -e "${YELLOW}🎮 Press Ctrl+C to stop both detectors${NC}"
            echo -e "${GREEN}✅ Running in headless mode - no GUI windows${NC}"
            wait
            ;;
            
        5)
            echo ""
            echo -e "${YELLOW}📋 System Status Check...${NC}"
            echo "========================="
            echo ""
            
            setup_environment
            
            echo -e "${BLUE}🔌 ROS2 Topics:${NC}"
            ros2 topic list | grep -E "(camera|apriltag|image)" | sort
            echo ""
            
            echo -e "${GREEN}📷 Camera Status:${NC}"
            timeout 3 ros2 topic hz /camera/image_raw 2>/dev/null || echo -e "${RED}❌ No camera data${NC}"
            echo ""
            
            echo -e "${BLUE}🏷️ AprilTag Detection Status:${NC}"
            timeout 3 ros2 topic hz /apriltag_detections 2>/dev/null || echo -e "${RED}❌ No AprilTag detection data${NC}"
            echo ""
            
            echo -e "${YELLOW}🔄 Running Processes:${NC}"
            check_processes
            echo ""
            
            echo -e "${GREEN}📦 Workspace Build Status:${NC}"
            if [ -f "install/warehouse_robot_system/lib/warehouse_robot_system/apriltag_detector_node" ]; then
                echo -e "   ${GREEN}✅ apriltag_detector_node built successfully${NC}"
            else
                echo -e "   ${RED}❌ apriltag_detector_node not found - rebuild needed${NC}"
            fi
            
            if [ -f "install/warehouse_robot_system/lib/warehouse_robot_system/colour_detector_node" ]; then
                echo -e "   ${GREEN}✅ colour_detector_node built successfully${NC}"
            else
                echo -e "   ${RED}❌ colour_detector_node not found - rebuild needed${NC}"
            fi
            ;;
            
        5)
            kill_detectors
            echo -e "${GREEN}✅ All detectors stopped${NC}"
            ;;
            
        6)
            kill_detectors
            echo -e "${GREEN}Goodbye! 👋${NC}"
            exit 0
            ;;
            
        *)
            echo -e "${RED}❌ Invalid choice. Please try again.${NC}"
            ;;
    esac
    
    echo ""
    echo -e "${YELLOW}Press Enter to continue...${NC}"
    read
done
