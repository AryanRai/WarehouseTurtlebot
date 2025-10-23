#!/bin/bash
# Quick start script for MTRX3760 Project 2

echo "🚀 MTRX3760 Project 2 - Quick Start"
echo "===================================="
echo ""

# Check if we're in the right directory
if [ ! -f "launch_mgen.sh" ]; then
    echo "❌ Please run this script from the project root directory"
    exit 1
fi

echo "🎯 What would you like to do?"
echo ""
echo "   1. 🛠️  First time setup (install dependencies)"
echo "   2. 🔨 Build the project"
echo "   3. 🧪 Test the warehouse system"
echo "   4. 🗺️  Run full SLAM demo"
echo "   5. 🤖 Quick SLAM simulation (requires existing Gazebo)"
echo "   6. 🎮 Robot teleop control"
echo "   7. ❓ Show help"
echo ""

read -p "Select option (1-7): " choice

case $choice in
    1)
        echo "🛠️  Running first time setup..."
        ./scripts/setup_dependencies.sh
        ;;
    2)
        echo "🔨 Building project..."
        ./scripts/build_project.sh
        ;;
    3)
        echo "🧪 Testing warehouse system..."
        ./scripts/test_warehouse_system.sh
        ;;
    4)
        echo "🗺️  Starting full SLAM demo..."
        echo ""
        echo "This will:"
        echo "   • Generate a maze"
        echo "   • Launch Gazebo"
        echo "   • Provide instructions for SLAM simulation"
        echo ""
        read -p "Continue? (y/n): " confirm
        if [[ $confirm == [yY] ]]; then
            ./scripts/run_full_slam_demo.sh
        fi
        ;;
    5)
        echo "🤖 Starting SLAM simulation..."
        echo ""
        echo "⚠️  Make sure Gazebo is already running!"
        echo "   (Use option 4 or run ./launch_mgen.sh first)"
        echo ""
        read -p "Continue? (y/n): " confirm
        if [[ $confirm == [yY] ]]; then
            ./scripts/run_slam_sim.sh
        fi
        ;;
    6)
        echo "🎮 Starting robot teleop..."
        ./scripts/run_teleop.sh
        ;;
    7)
        echo "❓ Help - Available Scripts:"
        echo ""
        echo "📁 Setup & Build:"
        echo "   ./scripts/setup_dependencies.sh - Install system dependencies"
        echo "   ./scripts/build_project.sh      - Build with Anaconda conflict fixes"
        echo ""
        echo "🧪 Testing:"
        echo "   ./scripts/test_warehouse_system.sh - Test warehouse robot system"
        echo ""
        echo "🤖 SLAM & Simulation:"
        echo "   ./scripts/run_full_slam_demo.sh - Complete demo with maze generation"
        echo "   ./scripts/run_slam_sim.sh       - SLAM simulation (needs Gazebo running)"
        echo "   ./scripts/spawn_robot.sh        - Spawn robot in existing simulation"
        echo ""
        echo "🎮 Control:"
        echo "   ./scripts/run_teleop.sh - Manual robot control"
        echo ""
        echo "📖 Documentation:"
        echo "   scripts/README.md - Detailed script documentation"
        echo "   README.md         - Project overview"
        echo ""
        echo "🔧 Troubleshooting:"
        echo "   • Anaconda conflicts: Use ./scripts/build_project.sh"
        echo "   • Missing deps: Run ./scripts/setup_dependencies.sh"
        echo "   • Build issues: Try ./scripts/build_project.sh clean"
        ;;
    *)
        echo "❌ Invalid option. Please select 1-7."
        exit 1
        ;;
esac

echo ""
echo "✅ Done! Run ./scripts/quick_start.sh again for more options."