#!/bin/bash
# System Status Checker
# Diagnoses common issues with the autonomous SLAM system

echo "🔍 Autonomous SLAM System Status Check"
echo "======================================="
echo ""

# Check if Gazebo is running
echo "1️⃣  Checking Gazebo..."
if pgrep -f "gz sim" > /dev/null || pgrep -f "gzserver" > /dev/null; then
    echo "   ✅ Gazebo is running"
    
    # Check if robot model is loaded
    if pgrep -f "spawn_turtlebot3" > /dev/null; then
        echo "   ✅ TurtleBot3 spawn process active"
    else
        echo "   ⚠️  TurtleBot3 spawn process not found"
    fi
else
    echo "   ❌ Gazebo is NOT running"
    echo "   💡 Start with: ./launch_warehouse.sh"
fi
echo ""

# Check if ROS bridge is running
echo "2️⃣  Checking ROS-Gazebo Bridge..."
if pgrep -f "ros_gz_bridge" > /dev/null; then
    echo "   ✅ ROS-Gazebo bridge is running"
else
    echo "   ❌ ROS-Gazebo bridge is NOT running"
fi
echo ""

# Check if SLAM Toolbox is running
echo "3️⃣  Checking SLAM Toolbox..."
if pgrep -f "slam_toolbox" > /dev/null; then
    echo "   ✅ SLAM Toolbox is running"
    
    # Try to determine mode
    if pgrep -f "localization" > /dev/null; then
        echo "   📍 Mode: LOCALIZATION (using existing map)"
    else
        echo "   🗺️  Mode: MAPPING (creating new map)"
    fi
else
    echo "   ❌ SLAM Toolbox is NOT running"
fi
echo ""

# Check if autonomous controller is running
echo "4️⃣  Checking Autonomous Controller..."
if pgrep -f "autonomous_slam_node" > /dev/null; then
    echo "   ✅ Autonomous SLAM node is running"
else
    echo "   ❌ Autonomous SLAM node is NOT running"
fi
echo ""

# Check if delivery robot is running
echo "5️⃣  Checking Delivery Robot..."
if pgrep -f "delivery_robot_node" > /dev/null; then
    echo "   ✅ Delivery robot node is running"
else
    echo "   ℹ️  Delivery robot node is not running (normal if in exploration mode)"
fi
echo ""

# Check ROS topics (requires ROS to be sourced)
echo "6️⃣  Checking ROS Topics..."
if command -v ros2 &> /dev/null; then
    # Source ROS
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null
    
    # Check cmd_vel
    if timeout 1 ros2 topic list 2>/dev/null | grep -q "/cmd_vel"; then
        echo "   ✅ /cmd_vel topic exists"
        
        # Check if messages are being published
        if timeout 2 ros2 topic hz /cmd_vel 2>/dev/null | grep -q "average rate"; then
            echo "   ✅ Robot is receiving velocity commands"
        else
            echo "   ⚠️  No velocity commands being published"
            echo "   💡 Robot may be idle or waiting for map/pose"
        fi
    else
        echo "   ❌ /cmd_vel topic not found"
    fi
    
    # Check map
    if timeout 1 ros2 topic list 2>/dev/null | grep -q "/map"; then
        echo "   ✅ /map topic exists"
    else
        echo "   ❌ /map topic not found"
    fi
    
    # Check pose
    if timeout 1 ros2 topic list 2>/dev/null | grep -q "/pose"; then
        echo "   ✅ /pose topic exists"
    else
        echo "   ⚠️  /pose topic not found"
    fi
else
    echo "   ⚠️  ROS 2 not found in PATH"
fi
echo ""

# Check for TF issues
echo "7️⃣  Checking TF Transforms..."
if command -v ros2 &> /dev/null; then
    source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash 2>/dev/null
    
    # Try to get a transform
    if timeout 2 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q "At time"; then
        echo "   ✅ TF transforms are working (map → base_footprint)"
    else
        echo "   ❌ TF transforms are broken or not available"
        echo "   💡 This is likely the cause of white/frozen robot"
        echo "   💡 Solution: Restart Gazebo and the autonomous script"
        echo "   💡 See: docs/TF_RESTART_FIX.md"
    fi
else
    echo "   ⚠️  Cannot check TF (ROS 2 not in PATH)"
fi
echo ""

# Summary
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📊 SUMMARY"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

GAZEBO_OK=$(pgrep -f "gz sim" > /dev/null && echo "yes" || echo "no")
SLAM_OK=$(pgrep -f "slam_toolbox" > /dev/null && echo "yes" || echo "no")
CONTROLLER_OK=$(pgrep -f "autonomous_slam_node\|delivery_robot_node" > /dev/null && echo "yes" || echo "no")

if [[ "$GAZEBO_OK" == "yes" ]] && [[ "$SLAM_OK" == "yes" ]] && [[ "$CONTROLLER_OK" == "yes" ]]; then
    echo "✅ System appears to be running normally"
    echo ""
    echo "If robot is white/frozen despite this:"
    echo "  → TF transform issue (see above)"
    echo "  → Restart Gazebo and autonomous script"
elif [[ "$GAZEBO_OK" == "no" ]]; then
    echo "❌ Gazebo is not running"
    echo ""
    echo "Start with: ./launch_warehouse.sh"
elif [[ "$SLAM_OK" == "no" ]]; then
    echo "❌ SLAM Toolbox is not running"
    echo ""
    echo "Start with: ./scripts/run_autonomous_slam.sh"
elif [[ "$CONTROLLER_OK" == "no" ]]; then
    echo "❌ No controller node is running"
    echo ""
    echo "Start with: ./scripts/run_autonomous_slam.sh"
else
    echo "⚠️  System status unclear"
    echo ""
    echo "Try restarting everything:"
    echo "  1. Stop all scripts (Ctrl+C)"
    echo "  2. ./launch_warehouse.sh"
    echo "  3. ./scripts/run_autonomous_slam.sh"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
