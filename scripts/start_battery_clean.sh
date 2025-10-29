#!/bin/bash
# Start battery nodes in a clean environment without conda interference

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

# Source ROS
source /opt/ros/jazzy/setup.bash

# Source workspace if provided
if [ ! -z "$1" ]; then
    cd "$1"
    source install/setup.bash
fi

# Get node name and config file from arguments
NODE_NAME="$2"
CONFIG_FILE="$3"

# Start the battery node
if [ ! -z "$CONFIG_FILE" ]; then
    exec ros2 run mtrx3760_battery "$NODE_NAME" --ros-args --params-file "$CONFIG_FILE"
else
    exec ros2 run mtrx3760_battery "$NODE_NAME"
fi
