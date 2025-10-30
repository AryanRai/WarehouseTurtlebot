#!/bin/bash
# Clean battery monitor launcher
# Starts battery monitor/simulator in a clean environment without conda interference
#
# Usage:
#   ./scripts/start_battery_clean.sh <workspace_path> <node_name> <config_file>
#
# Arguments:
#   workspace_path: Path to turtlebot3_ws
#   node_name: battery_monitor_node or battery_simulator_node
#   config_file: Path to battery_params.yaml

WORKSPACE_PATH="$1"
NODE_NAME="$2"
CONFIG_FILE="$3"

if [ -z "$WORKSPACE_PATH" ] || [ -z "$NODE_NAME" ] || [ -z "$CONFIG_FILE" ]; then
    echo "Usage: $0 <workspace_path> <node_name> <config_file>"
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

# Run the battery node
exec ros2 run mtrx3760_battery "$NODE_NAME" --ros-args --params-file "$CONFIG_FILE"
