#!/bin/bash
# Build script that handles Anaconda library conflicts

echo "🔨 Building MTRX3760 Project 2"
echo "==============================="
echo ""

cd "$(dirname "$0")/../turtlebot3_ws"

# Temporarily remove Anaconda from PATH to avoid library conflicts
echo "🔧 Configuring build environment..."
export PATH=$(echo $PATH | tr ':' '\n' | grep -v anaconda3 | tr '\n' ':')
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v anaconda3 | tr '\n' ':')

# Remove Anaconda from CMAKE paths
export CMAKE_PREFIX_PATH=$(echo $CMAKE_PREFIX_PATH | tr ':' '\n' | grep -v anaconda3 | tr '\n' ':')

echo "✅ Environment configured (Anaconda paths temporarily removed)"
echo ""

# Clean previous build if requested
if [ "$1" = "clean" ]; then
    echo "🧹 Cleaning previous build..."
    rm -rf build install log
    echo "✅ Clean complete"
    echo ""
fi

echo "🔨 Building packages..."
echo ""

# Build warehouse_robot_system first
echo "1️⃣ Building warehouse_robot_system..."
colcon build --packages-select warehouse_robot_system --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "❌ Failed to build warehouse_robot_system!"
    exit 1
fi

echo "✅ warehouse_robot_system built successfully"
echo ""

# Build turtlebot3_gazebo with library conflict fixes
echo "2️⃣ Building turtlebot3_gazebo..."
colcon build --packages-select turtlebot3_gazebo --allow-overriding turtlebot3_gazebo --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "❌ Failed to build turtlebot3_gazebo!"
    echo ""
    echo "🔍 Troubleshooting tips:"
    echo "   • Make sure you're not in a conda environment"
    echo "   • Try: conda deactivate"
    echo "   • Install missing dependencies:"
    echo "     sudo apt install libcurl4-openssl-dev"
    echo "     sudo apt install ros-humble-turtlebot3*"
    exit 1
fi

echo "✅ turtlebot3_gazebo built successfully"
echo ""

# Build remaining packages
echo "3️⃣ Building remaining packages..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    echo "⚠️  Some packages failed to build, but core packages are ready"
else
    echo "✅ All packages built successfully"
fi

echo ""
echo "🎉 Build Complete!"
echo ""
echo "🚀 Next steps:"
echo "   • Test the system: ./scripts/test_warehouse_system.sh"
echo "   • Run full demo: ./scripts/run_full_slam_demo.sh"
echo "   • Quick SLAM test: ./scripts/run_slam_sim.sh"
echo ""
echo "💡 Remember to source the workspace:"
echo "   source turtlebot3_ws/install/setup.bash"