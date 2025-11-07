#!/bin/bash
# Clean rebuild script - rebuilds warehouse_robot_system without conda interference
# This ensures the binaries don't have hardcoded anaconda library paths

echo " Clean Rebuild - warehouse_robot_system"
echo "=========================================="
echo ""

# Deactivate conda if active
if [ ! -z "$CONDA_PREFIX" ]; then
    echo "️  Conda environment detected: $CONDA_DEFAULT_ENV"
    echo "   Deactivating for clean build..."
    
    # Unset all conda-related environment variables
    unset CONDA_PREFIX
    unset CONDA_DEFAULT_ENV
    unset CONDA_PROMPT_MODIFIER
    unset CONDA_SHLVL
    unset CONDA_PYTHON_EXE
    unset CONDA_EXE
    unset _CE_CONDA
    unset _CE_M
    
    # Remove conda from PATH
    export PATH=$(echo $PATH | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
    
    # Remove conda from LD_LIBRARY_PATH if it exists
    if [ ! -z "$LD_LIBRARY_PATH" ]; then
        export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
    fi
    
    echo "    Conda deactivated"
    echo ""
fi

# Navigate to workspace
cd "$(dirname "$0")/../turtlebot3_ws" || exit 1

echo " Cleaning previous build..."
rm -rf build/warehouse_robot_system install/warehouse_robot_system

echo ""
echo " Building warehouse_robot_system..."
source /opt/ros/jazzy/setup.bash
colcon build --packages-select warehouse_robot_system

if [ $? -eq 0 ]; then
    echo ""
    echo " Clean build successful!"
    echo ""
    echo "Verifying binary doesn't link to anaconda..."
    if ldd install/warehouse_robot_system/lib/warehouse_robot_system/autonomous_slam_node | grep -q anaconda; then
        echo " WARNING: Binary still links to anaconda libraries!"
        echo "   You may need to start a completely fresh terminal without conda initialized."
    else
        echo " Binary is clean - no anaconda dependencies!"
    fi
else
    echo ""
    echo " Build failed!"
    exit 1
fi
