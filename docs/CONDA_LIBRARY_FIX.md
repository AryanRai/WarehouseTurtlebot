# Fixing Conda Library Conflicts with ROS 2

## Problem

When running ROS 2 nodes with Anaconda/Miniconda active, you may see errors like:
```
libstdc++.so.6: version `GLIBCXX_3.4.32' not found
libstdc++.so.6: version `GLIBCXX_3.4.30' not found
```

This happens because Conda's older C++ standard library conflicts with the system libraries that ROS 2 needs.

## Solution 1: Use the No-Conda Wrapper (Recommended)

The easiest solution is to use the provided wrapper script that automatically handles conda deactivation:

```bash
./run_slam_no_conda.sh
```

This script:
- Detects if conda is active
- Temporarily deactivates conda for the ROS session
- Cleans up PATH and LD_LIBRARY_PATH
- Runs the autonomous SLAM system in a clean environment

## Solution 2: Manual Conda Deactivation

Before running any ROS commands:

```bash
conda deactivate
./scripts/run_autonomous_slam.sh
```

## Solution 3: Permanent Fix (Advanced)

If you frequently work with ROS 2, consider adding this to your `~/.bashrc`:

```bash
# Function to run ROS commands without conda interference
ros_clean() {
    if [ ! -z "$CONDA_PREFIX" ]; then
        # Save conda state
        local SAVED_CONDA_ENV="$CONDA_DEFAULT_ENV"
        
        # Deactivate conda
        conda deactivate
        
        # Run command
        "$@"
        
        # Reactivate conda if it was active
        if [ ! -z "$SAVED_CONDA_ENV" ]; then
            conda activate "$SAVED_CONDA_ENV"
        fi
    else
        # No conda active, just run command
        "$@"
    fi
}
```

Then use: `ros_clean ./scripts/run_autonomous_slam.sh`

## How the Clean Launchers Work

The project includes clean launcher scripts for components that need them:
- `scripts/start_autonomous_slam_clean.sh` - Autonomous SLAM node
- `scripts/start_rosbridge_clean.sh` - ROSBridge WebSocket server
- `scripts/start_battery_clean.sh` - Battery monitoring

These scripts:
1. Unset all conda environment variables
2. Remove conda paths from PATH and LD_LIBRARY_PATH
3. Source ROS setup in clean environment
4. Execute the ROS node

## Verification

To verify conda is not interfering, check your library path:

```bash
# Should NOT contain anaconda/miniconda paths
echo $LD_LIBRARY_PATH

# Should use system libstdc++
ldd $(which ros2) | grep libstdc++
```

## Why This Happens

- ROS 2 Jazzy is built with GCC 13+ which provides GLIBCXX_3.4.32
- Conda typically ships with older GCC versions (GCC 11 or earlier)
- When conda is active, its library paths take precedence
- ROS 2 binaries fail to find the newer symbols they need

## Alternative: Conda-Forge ROS 2

If you need both conda and ROS 2 in the same environment, consider using conda-forge's ROS 2 packages:

```bash
conda create -n ros2_env
conda activate ros2_env
conda install -c conda-forge ros-jazzy-desktop
```

However, this is not recommended for this project as it may have compatibility issues with the existing setup.
