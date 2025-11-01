#!/bin/bash
# TurtleBot Camera Calibration System
# Workaround for OpenCV GUI library conflicts

export ROS_DOMAIN_ID=29
export TURTLEBOT3_MODEL=burger

# Fix library path conflicts
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
unset SNAP

echo "🎨 TurtleBot Camera Calibration System"
echo "======================================"
echo ""
echo "🔧 Prerequisites:"
echo "  1. SSH into TurtleBot: ssh -X ubuntu@TURTLEBOT_IP"
echo "  2. Run: ros2 launch turtlebot3_bringup robot.launch.py"
echo "  3. Verify camera: ros2 topic hz /camera/image_raw"
echo ""
echo "📖 How HSV Calibration Works:"
echo "  • Detects 16h5 AprilTags in camera feed"
echo "  • Creates sampling regions around each tag (above/below/left/right)"
echo "  • Analyzes HSV color values in each region"
echo "  • Classifies colors: Green=Mould, Blue=Water, Red=Blood"
echo "  • Shows live overlay with colored rectangles and HSV stats"
echo ""
echo "Choose mode:"
echo "  1) 🎥 Interactive Calibration (Live GUI)"
echo "  2) 📁 File-based Calibration (Save images)" 
echo "  3) 🔍 Normal Detection Mode"
echo "  4) 📋 Check System Status"
echo "  5) 🚪 Exit"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
  1)
    echo ""
    echo "🎥 Starting Interactive Calibration..."
    echo "======================================"
    echo ""
    echo "This will attempt to open an OpenCV window showing:"
    echo "  📹 Live camera feed"
    echo "  🎯 AprilTag detection overlays"
    echo "  🟩🟦🟥 Colored sampling rectangles"
    echo "  📊 Real-time HSV values"
    echo ""
    echo "Controls:"
    echo "  's' = Save calibration to ~/hsv_calibration.yaml"
    echo "  'c' = Print HSV values to terminal"
    echo "  'q' = Quit"
    echo ""
    
    # Set up environment to avoid ROS logging errors
    export ROS_LOG_DIR=/tmp/ros_logs
    mkdir -p $ROS_LOG_DIR
    export RCUTILS_LOGGING_USE_STDOUT=1
    export RCUTILS_COLORIZED_OUTPUT=1
    
    # Try the GUI version with library fixes
    echo "Starting colour detector with GUI and AprilTag detection..."
    cd ~/MTRX3760_Project_2/turtlebot3_ws
    source install/setup.bash
    
    # Fix X11 and Qt display issues for GUI mode
    echo "🔧 Setting up display environment..."
    
    # Check if we're in a container or remote session
    if [ -n "$SSH_CONNECTION" ]; then
        echo "   Detected SSH session - ensuring X11 forwarding"
        xhost +local: 2>/dev/null || echo "   Warning: xhost not available"
    fi
    
    # Set up proper Qt platform
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export DISPLAY=${DISPLAY:-:0}
    
    # Create a clean environment for OpenCV/Qt
    echo "   Using display: $DISPLAY"
    echo "   Testing X11 connection..."
    if command -v xdpyinfo >/dev/null 2>&1; then
        if xdpyinfo >/dev/null 2>&1; then
            echo "   ✅ X11 connection successful"
        else
            echo "   ❌ X11 connection failed - falling back to file mode"
            export DISPLAY=""
        fi
    else
        echo "   ⚠️  xdpyinfo not available - testing with simple X call"
        if python3 -c "import os; os.system('python3 -c \"import tkinter; tkinter.Tk().destroy()\"')" 2>/dev/null; then
            echo "   ✅ Basic GUI test passed"
        else
            echo "   ❌ GUI test failed - using file mode"
            export DISPLAY=""
        fi
    fi
    
    ros2 run warehouse_robot_system colour_detector_node --ros-args -p calibration_mode:=true
    ;;
    
  2)
    echo ""
    echo "📁 Starting File-based Calibration..."
    echo "====================================="
    echo ""
    echo "Since GUI has library conflicts, images will be saved to /tmp/"
    echo "View with: feh /tmp/calibration_frame_*.jpg"
    echo ""
    
    cd ~/MTRX3760_Project_2/turtlebot3_ws
    source install/setup.bash
    
    # Set up environment to avoid ROS logging errors  
    export ROS_LOG_DIR=/tmp/ros_logs
    mkdir -p $ROS_LOG_DIR
    export RCUTILS_LOGGING_USE_STDOUT=1
    export RCUTILS_COLORIZED_OUTPUT=1
    
    # Run without GUI - will save files instead
    echo "Running file-based calibration with AprilTag detection..."
    DISPLAY="" ros2 run warehouse_robot_system colour_detector_node \
      --ros-args -p calibration_mode:=true
      
    ;;
    
  3)
    echo ""
    echo "🔍 Starting Normal Detection Mode..."
    echo "==================================="
    echo ""
    
    cd ~/MTRX3760_Project_2/turtlebot3_ws
    source install/setup.bash
    
    # Set up environment to avoid ROS logging errors
    export ROS_LOG_DIR=/tmp/ros_logs  
    mkdir -p $ROS_LOG_DIR
    export RCUTILS_LOGGING_USE_STDOUT=1
    export RCUTILS_COLORIZED_OUTPUT=1
    
    echo "Running normal detection mode with AprilTag detection..."
    ros2 run warehouse_robot_system colour_detector_node \
      --ros-args -p calibration_mode:=false
      
    ;;
    
  4)
    echo ""
    echo "📋 System Status Check..."
    echo "========================="
    echo ""
    
    cd ~/MTRX3760_Project_2/turtlebot3_ws
    source install/setup.bash
    
    echo "🔌 ROS2 Topics:"
    ros2 topic list | grep -E "(camera|apriltag|image)" | sort
    echo ""
    
    echo "📷 Camera Status:"
    timeout 3 ros2 topic hz /camera/image_raw 2>/dev/null || echo "❌ No camera data"
    echo ""
    
    echo "🏷️ AprilTag Detection:"
    echo "✅ Built-in AprilTag detection (16h5 family) in colour_detector_node"
    echo ""
    
    echo "📦 Workspace Build Status:"
    if [ -f "install/warehouse_robot_system/lib/warehouse_robot_system/colour_detector_node" ]; then
        echo "✅ colour_detector_node built successfully"
    else
        echo "❌ colour_detector_node not found - rebuild needed"
    fi
    ;;
    
  5)
    echo "Goodbye! 👋"
    exit 0
    ;;
    
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac
