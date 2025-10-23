#!/bin/bash
# Test script for Warehouse Robot System

echo "🏭 Testing Warehouse Robot System"
echo "================================="
echo ""

cd "$(dirname "$0")/../turtlebot3_ws"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built! Building now..."
    cd ..
    ./scripts/build_project.sh
    if [ $? -ne 0 ]; then
        echo "❌ Build failed!"
        exit 1
    fi
    cd turtlebot3_ws
fi

source install/setup.bash

echo "🧪 Running Warehouse Robot System Tests..."
echo ""

echo "1️⃣ Testing SLAM Module..."
ros2 run warehouse_robot_system slam_test
echo ""

echo "2️⃣ Testing Main Warehouse System (5 second demo)..."
timeout 5s ros2 run warehouse_robot_system warehouse_robot_main
echo ""

echo "✅ Warehouse Robot System Tests Complete!"
echo ""
echo "📊 Test Results:"
echo "   • SLAM module: Grid operations, coordinate transforms, frontier detection"
echo "   • Main system: Polymorphic robot creation, battery management, factory pattern"
echo ""
echo "🚀 To run full SLAM simulation with RViz:"
echo "   ./scripts/run_slam_sim.sh"