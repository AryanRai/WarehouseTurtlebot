#!/bin/bash
# Camera Calibration Menu System
# Runs camera detection in different modes on TurtleBot

export ROS_DOMAIN_ID=29
export TURTLEBOT3_MODEL=burger

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
cd ~/MTRX3760_Project_2/turtlebot3_ws
source install/setup.bash
cd -

echo " Camera Calibration System"
echo "============================"
echo ""
echo "Prerequisites:"
echo "  • SSH into TurtleBot: ssh -X ubuntu@10.42.0.1"
echo "  • Run bringup: ros2 launch turtlebot3_bringup robot.launch.py"
echo "  • Camera topics should be available: /camera/image_raw"
echo ""
echo "Choose an option:"
echo "  1)  Calibrate HSV colours (LIVE camera window)"
echo "  2)  Test detection (normal mode)"
echo "  3)  Check camera topics"
echo "  4)  View saved calibration"
echo "  5)  Exit"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
  1)
    echo ""
    echo "Starting HSV Calibration Mode..."
    echo "================================="
    echo ""
    echo "Instructions:"
    echo "  • Since GUI is disabled due to library conflicts, calibration"
    echo "    images will be saved to /tmp/ directory"
    echo "  • View saved images: /tmp/calibration_frame_*.jpg"
    echo "  • Raw camera images: /tmp/raw_frame_*.jpg"
    echo "  • Use 'feh /tmp/calibration_frame_*.jpg' or similar to view"
    echo "  • AprilTag detections and camera feed both required"
    echo ""
    echo "Starting colour detector in file-based calibration mode..."
    ros2 run warehouse_robot_system colour_detector_node \
      --ros-args -p calibration_mode:=true
    ;;
    
  2)
    echo ""
    echo "Starting Detection Test Mode..."
    echo "==============================="
    echo ""
    ros2 run warehouse_robot_system colour_detector_node \
      --ros-args -p calibration_mode:=false
    ;;
    
  3)
    echo ""
    echo "Demo Mode - Testing with Synthetic Data"
    echo "========================================"
    echo ""
    echo "This will:"
    echo "  1. Start colour detector in calibration mode"
    echo "  2. Publish test camera image with colored patches"
    echo "  3. Publish test AprilTag detection"
    echo "  4. Show resulting calibration images in /tmp/"
    echo ""
    
    echo "Starting colour detector..."
    ros2 run warehouse_robot_system colour_detector_node \
      --ros-args -p calibration_mode:=true &
    DETECTOR_PID=$!
    sleep 2
    
    echo "Publishing test AprilTag detection..."
    ros2 topic pub --once /apriltag/detections apriltag_msgs/msg/AprilTagDetectionArray \
      "{header: {frame_id: 'camera_frame'}, detections: [{id: 0, family: 'tag36h11', centre: {x: 320.0, y: 240.0}, corners: [{x: 300.0, y: 220.0}, {x: 340.0, y: 220.0}, {x: 340.0, y: 260.0}, {x: 300.0, y: 260.0}]}]}" &
    
    echo "The system is now running. Press Ctrl+C to stop..."
    wait $DETECTOR_PID
    ;;
    
  4)
    echo ""
    echo "Saved Calibration:"
    echo "=================="
    if [ -f ~/hsv_calibration.yaml ]; then
      cat ~/hsv_calibration.yaml
    else
      echo "No calibration file found at ~/hsv_calibration.yaml"
    fi
    echo ""
    echo "Saved calibration images:"
    ls -la /tmp/calibration_frame_*.jpg 2>/dev/null || echo "No calibration images found"
    ;;
    
  5)
    echo "Exiting..."
    exit 0
    ;;
    
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac