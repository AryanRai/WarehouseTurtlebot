#!/bin/bash
# Quick build script for warehouse robot system

cd "$(dirname "$0")/../turtlebot3_ws"

echo "🔨 Building warehouse_robot_system..."
echo ""

colcon build --packages-select warehouse_robot_system --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo ""
    echo "To use the new zone definition mode:"
    echo "  ./scripts/run_autonomous_slam.sh -preload"
    echo "  Select option 1: DEFINE DELIVERY ZONES"
else
    echo ""
    echo "❌ Build failed! Check errors above."
    exit 1
fi
