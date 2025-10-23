#!/bin/bash
# Setup script for MTRX3760 Project 2 dependencies

echo "📦 Setting up MTRX3760 Project 2 Dependencies"
echo "=============================================="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "❌ Please don't run this script as root (no sudo needed for most steps)"
    exit 1
fi

echo "🔍 Checking system requirements..."

# Check ROS2 installation
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found! Please install ROS2 Humble first:"
    echo "   https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✅ ROS2 found"

# Check if we need to install TurtleBot3 packages
echo ""
echo "📦 Installing TurtleBot3 packages..."
sudo apt update
sudo apt install -y \
    ros-humble-turtlebot3* \
    ros-humble-cartographer* \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    libcurl4-openssl-dev \
    pkg-config

if [ $? -ne 0 ]; then
    echo "❌ Failed to install some packages. Please check your internet connection."
    exit 1
fi

echo "✅ TurtleBot3 packages installed"

# Set up environment variables
echo ""
echo "🔧 Setting up environment..."

# Check if TURTLEBOT3_MODEL is already set
if ! grep -q "TURTLEBOT3_MODEL" ~/.bashrc; then
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    echo "✅ Added TURTLEBOT3_MODEL to ~/.bashrc"
else
    echo "✅ TURTLEBOT3_MODEL already configured"
fi

# Set for current session
export TURTLEBOT3_MODEL=burger

# Initialize submodules if needed
echo ""
echo "📁 Checking submodules..."
cd "$(dirname "$0")/.."

if [ -f ".gitmodules" ]; then
    git submodule update --init --recursive
    echo "✅ Submodules initialized"
else
    echo "ℹ️  No submodules found"
fi

echo ""
echo "🎉 Dependencies setup complete!"
echo ""
echo "🚀 Next steps:"
echo "   1. Restart your terminal or run: source ~/.bashrc"
echo "   2. Build the project: ./scripts/build_project.sh"
echo "   3. Test the system: ./scripts/test_warehouse_system.sh"
echo ""
echo "💡 If you encounter build issues:"
echo "   • Make sure you're not in a conda environment (conda deactivate)"
echo "   • Try building with: ./scripts/build_project.sh clean"