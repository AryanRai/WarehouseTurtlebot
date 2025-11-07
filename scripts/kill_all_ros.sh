#!/bin/bash
# Kill All ROS Nodes Script
# Comprehensive cleanup of all ROS2 processes, Gazebo, and related components

echo " Killing All ROS Nodes and Related Processes"
echo "=============================================="
echo ""

# Function to kill processes by name with confirmation
kill_processes() {
    local process_name="$1"
    local display_name="$2"
    
    local pids=$(pgrep -f "$process_name" 2>/dev/null)
    if [ ! -z "$pids" ]; then
        echo " Killing $display_name processes..."
        echo "   PIDs: $pids"
        pkill -f "$process_name" 2>/dev/null
        sleep 1
        
        # Force kill if still running
        local remaining_pids=$(pgrep -f "$process_name" 2>/dev/null)
        if [ ! -z "$remaining_pids" ]; then
            echo "   Force killing remaining $display_name processes..."
            pkill -9 -f "$process_name" 2>/dev/null
        fi
        echo "    $display_name processes terminated"
    else
        echo "   ️  No $display_name processes found"
    fi
}

# Function to kill processes by exact command name
kill_by_command() {
    local command_name="$1"
    local display_name="$2"
    
    local pids=$(pgrep "$command_name" 2>/dev/null)
    if [ ! -z "$pids" ]; then
        echo " Killing $display_name..."
        echo "   PIDs: $pids"
        pkill "$command_name" 2>/dev/null
        sleep 1
        
        # Force kill if still running
        local remaining_pids=$(pgrep "$command_name" 2>/dev/null)
        if [ ! -z "$remaining_pids" ]; then
            echo "   Force killing remaining $display_name..."
            pkill -9 "$command_name" 2>/dev/null
        fi
        echo "    $display_name terminated"
    else
        echo "   ️  No $display_name processes found"
    fi
}

echo " Scanning for ROS2 and related processes..."
echo ""

# Kill ROS2 processes
echo "1 ROS2 Core Processes"
kill_processes "ros2" "ROS2 nodes"
kill_processes "_ros2_daemon" "ROS2 daemon"
kill_processes "ros2 daemon" "ROS2 daemon"

echo ""
echo "2 Specific ROS2 Nodes"
# Kill specific ROS2 nodes
kill_processes "autonomous_slam_main" "Autonomous SLAM Controller"
kill_processes "warehouse_robot_main" "Warehouse Robot System"
kill_processes "slam_test" "SLAM Test"
kill_processes "CTurtlebot3Drive_node" "TurtleBot3 Drive Node"
kill_processes "robot_state_publisher" "Robot State Publisher"
kill_processes "cartographer" "Cartographer SLAM"
kill_processes "turtlebot3_teleop" "TurtleBot3 Teleop"

echo ""
echo "3 Gazebo Processes"
# Kill Gazebo processes
kill_processes "gz sim" "Gazebo Sim"
kill_processes "gzserver" "Gazebo Server"
kill_processes "gzclient" "Gazebo Client"
kill_processes "gazebo" "Gazebo"
kill_by_command "gz" "Gazebo GZ"

echo ""
echo "4 Visualization Tools"
# Kill visualization tools
kill_by_command "rviz2" "RViz2"
kill_by_command "rqt" "RQT"
kill_processes "foxglove" "Foxglove Studio"

echo ""
echo "5 TurtleBot3 Specific Processes"
# Kill TurtleBot3 specific processes
kill_processes "turtlebot3" "TurtleBot3 processes"
kill_processes "spawn_turtlebot3" "TurtleBot3 Spawn"

echo ""
echo "6 Navigation and SLAM"
# Kill navigation and SLAM processes
kill_processes "nav2" "Navigation2"
kill_processes "navigation2" "Navigation2"
kill_processes "slam_toolbox" "SLAM Toolbox"
kill_processes "map_server" "Map Server"
kill_processes "amcl" "AMCL"

echo ""
echo "7 Transform and Sensor Processes"
# Kill transform and sensor processes
kill_processes "tf2" "TF2 processes"
kill_processes "static_transform_publisher" "Static Transform Publisher"
kill_processes "laser" "Laser processes"
kill_processes "lidar" "LiDAR processes"

echo ""
echo "8 Python ROS Processes"
# Kill Python ROS processes (if any)
kill_processes "python.*ros" "Python ROS processes"
kill_processes "python3.*ros" "Python3 ROS processes"

echo ""
echo "9 Cleanup ROS2 Environment"
# Clean up ROS2 environment
echo " Cleaning ROS2 environment..."

# Remove ROS2 daemon files if they exist
if [ -d "/tmp/ros2_daemon_*" ]; then
    echo "   Removing ROS2 daemon temporary files..."
    rm -rf /tmp/ros2_daemon_* 2>/dev/null
fi

# Clean up shared memory
if command -v ipcs >/dev/null 2>&1; then
    echo "   Cleaning shared memory segments..."
    # Remove shared memory segments created by ROS2
    ipcs -m | grep $(whoami) | awk '{print $2}' | xargs -r ipcrm -m 2>/dev/null || true
fi

# Kill any remaining processes that might be hanging
echo ""
echo " Final Cleanup"
echo " Checking for remaining processes..."

# Check for any remaining ROS-related processes
remaining_ros=$(pgrep -f "ros2\|gazebo\|gz\|rviz\|cartographer\|turtlebot3" 2>/dev/null || true)
if [ ! -z "$remaining_ros" ]; then
    echo "️  Found remaining ROS-related processes:"
    ps -p $remaining_ros -o pid,ppid,cmd 2>/dev/null || true
    echo ""
    read -p "Force kill remaining processes? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo " Force killing remaining processes..."
        echo "$remaining_ros" | xargs -r kill -9 2>/dev/null || true
        echo "    Force kill complete"
    fi
else
    echo "    No remaining ROS processes found"
fi

echo ""
echo " Process Summary"
echo "=================="

# Show summary of what's still running
echo " Current ROS-related processes:"
ros_processes=$(pgrep -f "ros2\|gazebo\|gz\|rviz\|cartographer\|turtlebot3" 2>/dev/null || true)
if [ -z "$ros_processes" ]; then
    echo "    No ROS-related processes running"
else
    echo "   ️  Still running:"
    ps -p $ros_processes -o pid,cmd --no-headers 2>/dev/null || true
fi

echo ""
echo " Additional Cleanup Commands (if needed):"
echo "   • Reset ROS2 daemon: ros2 daemon stop && ros2 daemon start"
echo "   • Clear ROS logs: rm -rf ~/.ros/log/*"
echo "   • Restart terminal or source workspace again"
echo ""
echo " ROS Cleanup Complete!"
echo ""
echo " To restart ROS2 services:"
echo "   ./scripts/run_autonomous_slam.sh"
echo "   or"
echo "   ./scripts/run_slam_sim.sh"