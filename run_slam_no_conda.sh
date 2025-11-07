#!/bin/bash
# Wrapper script to run autonomous SLAM without conda interference
# This ensures all ROS nodes start in a clean environment

echo " Deactivating conda environment..."
echo ""

# Deactivate conda if active
if [ ! -z "$CONDA_PREFIX" ]; then
    echo "   Detected conda environment: $CONDA_DEFAULT_ENV"
    echo "   Deactivating for clean ROS environment..."
    
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
else
    echo "   No conda environment detected"
    echo ""
fi

# Run the autonomous SLAM script with all arguments passed through
exec "$(dirname "$0")/scripts/run_autonomous_slam.sh" "$@"
