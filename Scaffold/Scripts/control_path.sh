#!/bin/bash
# Helper script to control path visualization recording

case "$1" in
  start|enable|on)
    echo "🟢 Enabling path recording..."
    ros2 service call /path_visualizer/toggle_recording std_srvs/srv/SetBool "{data: true}"
    ;;
  stop|disable|off)
    echo "🔴 Disabling path recording..."
    ros2 service call /path_visualizer/toggle_recording std_srvs/srv/SetBool "{data: false}"
    ;;
  clear|reset)
    echo "🗑️  Clearing path..."
    ros2 service call /path_visualizer/clear_path std_srvs/srv/Empty
    ;;
  *)
    echo "Path Visualizer Control"
    echo ""
    echo "Usage: ./control_path.sh [command]"
    echo ""
    echo "Commands:"
    echo "  start   - Start recording the robot's path"
    echo "  stop    - Stop recording (keeps existing path)"
    echo "  clear   - Clear the entire path"
    echo ""
    echo "Example workflow:"
    echo "  1. Let robot run for a while to stabilize SLAM"
    echo "  2. ./control_path.sh start"
    echo "  3. Watch the clean, drift-free path appear!"
    echo "  4. ./control_path.sh stop (when done)"
    echo "  5. ./control_path.sh clear (to start fresh)"
    ;;
esac
