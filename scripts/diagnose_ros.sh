#!/bin/bash
# ROS2 Diagnostic Script
# Helps diagnose common ROS2 and SLAM issues

echo "🔍 ROS2 System Diagnostics"
echo "=========================="
echo ""

cd "$(dirname "$0")/../turtlebot3_ws"

# Check if workspace is built and sourced
if [ ! -d "install" ]; then
    echo "❌ Workspace not built!"
    echo "   Run: ./scripts/build_project.sh"
    exit 1
fi

source install/setup.bash

echo "✅ Workspace found and sourced"
echo ""

echo "📊 ROS2 System Status"
echo "===================="

# Check ROS2 daemon
echo "🔧 ROS2 Daemon Status:"
if pgrep -f "_ros2_daemon" > /dev/null; then
    echo "   ✅ ROS2 daemon is running"
else
    echo "   ⚠️  ROS2 daemon not running - starting..."
    ros2 daemon start
fi

echo ""

# List active nodes
echo "🤖 Active ROS2 Nodes:"
nodes=$(ros2 node list 2>/dev/null)
if [ -z "$nodes" ]; then
    echo "   ❌ No ROS2 nodes found"
else
    echo "$nodes" | sed 's/^/   ✅ /'
fi

echo ""

# Check topics
echo "📡 Active Topics:"
topics=$(ros2 topic list 2>/dev/null)
if [ -z "$topics" ]; then
    echo "   ❌ No topics found"
else
    echo "   Key topics:"
    echo "$topics" | grep -E "(cmd_vel|scan|map|odom|tf)" | sed 's/^/   ✅ /' || echo "   ⚠️  No key topics found"
fi

echo ""

# Check transforms
echo "🔄 Transform Tree:"
if command -v ros2 >/dev/null 2>&1; then
    echo "   Checking for key frames..."
    
    # Check if tf2 is working
    if timeout 3s ros2 run tf2_ros tf2_echo map base_footprint >/dev/null 2>&1; then
        echo "   ✅ map -> base_footprint transform available"
    else
        echo "   ❌ map -> base_footprint transform NOT available"
        echo "      This is normal if SLAM hasn't started yet"
    fi
    
    if timeout 3s ros2 run tf2_ros tf2_echo base_footprint base_scan >/dev/null 2>&1; then
        echo "   ✅ base_footprint -> base_scan transform available"
    else
        echo "   ❌ base_footprint -> base_scan transform NOT available"
        echo "      Check if robot_state_publisher is running"
    fi
fi

echo ""

# Check specific processes
echo "🎯 Process Status:"
processes=(
    "gazebo:Gazebo Simulation"
    "gz sim:Gazebo Sim"
    "rviz2:RViz2 Visualization"
    "cartographer:Cartographer SLAM"
    "robot_state_publisher:Robot State Publisher"
    "autonomous_slam:Autonomous SLAM Controller"
)

for process_info in "${processes[@]}"; do
    IFS=':' read -r process_name display_name <<< "$process_info"
    if pgrep -f "$process_name" > /dev/null; then
        echo "   ✅ $display_name is running"
    else
        echo "   ❌ $display_name is NOT running"
    fi
done

echo ""

# Check topic rates (if nodes are running)
if [ ! -z "$topics" ]; then
    echo "📈 Topic Rates (sampling for 3 seconds):"
    
    key_topics=("/scan" "/cmd_vel" "/map" "/odom")
    for topic in "${key_topics[@]}"; do
        if echo "$topics" | grep -q "^$topic$"; then
            echo -n "   $topic: "
            rate=$(timeout 3s ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "N/A")
            if [ "$rate" != "N/A" ] && [ ! -z "$rate" ]; then
                echo "✅ ${rate} Hz"
            else
                echo "❌ No data"
            fi
        else
            echo "   $topic: ❌ Not available"
        fi
    done
fi

echo ""

# Check for common issues
echo "🚨 Common Issues Check:"

# Check for Anaconda conflicts
if echo $PATH | grep -q anaconda; then
    echo "   ⚠️  Anaconda detected in PATH - may cause library conflicts"
    echo "      Consider: conda deactivate"
else
    echo "   ✅ No Anaconda conflicts detected"
fi

# Check for multiple Gazebo instances
gazebo_count=$(pgrep -f "gazebo\|gz sim" | wc -l)
if [ "$gazebo_count" -gt 1 ]; then
    echo "   ⚠️  Multiple Gazebo instances detected ($gazebo_count)"
    echo "      Consider: ./scripts/kill_all_ros.sh"
elif [ "$gazebo_count" -eq 1 ]; then
    echo "   ✅ Single Gazebo instance running"
else
    echo "   ❌ No Gazebo instance running"
fi

# Check RViz message queue issues
if pgrep rviz2 > /dev/null; then
    rviz_errors=$(journalctl --since "1 minute ago" -u user@$(id -u).service 2>/dev/null | grep -i "message filter dropping" | wc -l)
    if [ "$rviz_errors" -gt 0 ]; then
        echo "   ⚠️  RViz message queue issues detected ($rviz_errors recent drops)"
        echo "      This is usually not critical but indicates high message load"
    else
        echo "   ✅ No RViz message queue issues"
    fi
fi

echo ""

# Recommendations
echo "💡 Recommendations:"

if [ -z "$nodes" ]; then
    echo "   🚀 No ROS nodes running - start with:"
    echo "      ./scripts/run_autonomous_slam.sh"
elif ! echo "$topics" | grep -q "/map"; then
    echo "   🗺️  No map topic - SLAM may not be running"
    echo "      Check if Cartographer is started"
elif ! pgrep -f "autonomous_slam" > /dev/null; then
    echo "   🤖 Autonomous SLAM controller not running"
    echo "      Start with: ros2 run warehouse_robot_system autonomous_slam_main"
else
    echo "   ✅ System appears to be running normally"
    echo "   📊 Monitor with: ros2 topic echo /cmd_vel"
fi

echo ""
echo "🔧 Useful Commands:"
echo "   • View all topics: ros2 topic list"
echo "   • Monitor velocity: ros2 topic echo /cmd_vel"
echo "   • Check transforms: ros2 run tf2_tools view_frames"
echo "   • Kill everything: ./scripts/kill_all_ros.sh"
echo "   • Restart system: ./scripts/run_autonomous_slam.sh"