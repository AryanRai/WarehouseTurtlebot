#!/bin/bash
# Complete SLAM Demo with Maze Generation
# This script generates a maze and launches Gazebo for SLAM testing

echo "🏗️  MTRX3760 Project 2 - Full SLAM Demo"
echo "======================================="
echo ""

# Check if we're in the right directory
if [ ! -f "launch_mgen.sh" ]; then
    echo "❌ Please run this script from the project root directory"
    echo "   (where launch_mgen.sh is located)"
    exit 1
fi

# Set up environment
export TURTLEBOT3_MODEL=burger

echo "🎯 Demo Options:"
echo "   1. Easy maze (10x10)"
echo "   2. Medium maze (15x15)"
echo "   3. Hard maze (20x20)"
echo "   4. Custom size"
echo ""

read -p "Select difficulty (1-4): " choice

case $choice in
    1)
        SIZE=10
        echo "🟢 Easy maze selected (${SIZE}x${SIZE})"
        ;;
    2)
        SIZE=15
        echo "🟡 Medium maze selected (${SIZE}x${SIZE})"
        ;;
    3)
        SIZE=20
        echo "🔴 Hard maze selected (${SIZE}x${SIZE})"
        ;;
    4)
        read -p "Enter maze size (10-30): " SIZE
        if [[ $SIZE -lt 10 || $SIZE -gt 30 ]]; then
            echo "❌ Invalid size. Using default (15x15)"
            SIZE=15
        fi
        echo "🎨 Custom maze selected (${SIZE}x${SIZE})"
        ;;
    *)
        SIZE=15
        echo "🟡 Default medium maze selected (${SIZE}x${SIZE})"
        ;;
esac

echo ""
echo "🏗️  Generating maze and launching Gazebo..."
echo "   This may take a moment..."
echo ""

# Generate and launch maze
./launch_mgen.sh $SIZE $SIZE

if [ $? -ne 0 ]; then
    echo "❌ Failed to launch maze!"
    exit 1
fi

echo ""
echo "✅ Maze generated and Gazebo launched!"
echo ""
echo "🚀 Next Steps:"
echo ""
echo "   Terminal 2 - Start SLAM simulation:"
echo "   ./scripts/run_slam_sim.sh"
echo ""
echo "   Terminal 3 - Control the robot:"
echo "   ./scripts/run_teleop.sh"
echo ""
echo "   Or test warehouse system:"
echo "   ./scripts/test_warehouse_system.sh"
echo ""
echo "📊 What you'll see:"
echo "   • Gazebo with generated maze"
echo "   • TurtleBot3 robot spawned in maze"
echo "   • RViz showing real-time SLAM mapping"
echo "   • Warehouse robot system demonstrating polymorphic behavior"
echo ""
echo "💡 Tips:"
echo "   • Move robot around to build complete map"
echo "   • Watch for frontier detection and path planning"
echo "   • Monitor warehouse system output for robot type switching"
echo ""
echo "Press Enter to continue or Ctrl+C to exit..."
read