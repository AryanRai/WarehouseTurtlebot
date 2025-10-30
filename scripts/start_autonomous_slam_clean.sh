#!/bin/bash
# Clean autonomous SLAM launcher
# Starts autonomous SLAM node in a clean environment without conda interference
#
# Usage:
#   ./scripts/start_autonomous_slam_clean.sh <workspace_path>
#
# Arguments:
#   workspace_path: Path to turtlebot3_ws

WORKSPACE_PATH="$1"

if [ -z "$WORKSPACE_PATH" ]; then
    echo "Usage: $0 <workspace_path>"
    exit 1
fi

# Deactivate conda if active
if [ ! -z "$CONDA_PREFIX" ]; then
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
fi

# Source ROS setup
cd "$WORKSPACE_PATH"
source install/setup.bash

# Run autonomous SLAM node
exec ros2 run warehouse_robot_system autonomous_slam_node
