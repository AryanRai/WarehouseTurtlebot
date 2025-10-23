# MTRX3760 Project 2 - Scripts

This folder contains utility scripts for running the warehouse robot system with SLAM capabilities.

## Available Scripts

### 🛠️ Build
- **`build_project.sh`** - Build the project with Anaconda conflict resolution
  - Temporarily removes Anaconda from build environment
  - Builds packages in correct order
  - Handles library path conflicts
  - Supports clean builds with `./scripts/build_project.sh clean`

### 🤖 SLAM Simulation
- **`run_autonomous_slam.sh`** - Autonomous SLAM with frontier exploration
  - Complete autonomous mapping system
  - Frontier detection and exploration
  - A* path planning and navigation
  - Returns to origin when mapping complete
  - Transitions to operational mode

- **`spawn_robot.sh`** - Spawn TurtleBot3 in existing Gazebo simulation
  - Spawns robot at specified coordinates
  - Starts robot_state_publisher
  - Validates Gazebo is running

### 🎮 Robot Control
- **`run_teleop.sh`** - Manual robot control via keyboard
  - Simple teleop interface for TurtleBot3
  - Use w/x for linear velocity, a/d for angular velocity
  - Space/s for emergency stop

### 🛑 Cleanup & Kill Scripts
- **`kill_all_ros.sh`** - Comprehensive ROS2 process cleanup
  - Graceful shutdown of all ROS2 nodes
  - Kills Gazebo, RViz, and related processes
  - Cleans up shared memory and temporary files
  - Shows summary of remaining processes

- **`kill_ros_force.sh`** - Force kill all ROS processes
  - Aggressive immediate termination
  - Use when normal shutdown fails
  - Nuclear option for stuck processes

### 🔍 Diagnostics
- **`diagnose_ros.sh`** - Comprehensive ROS2 system diagnostics
  - Checks running nodes and topics
  - Validates transforms and message rates
  - Identifies common issues
  - Provides troubleshooting recommendations

## Usage Examples

### Build the Project
```bash
# Build all packages
./scripts/build_project.sh

# Clean build if needed
./scripts/build_project.sh clean
```

### Autonomous SLAM
```bash
# Terminal 1: Generate and launch maze
./launch_mgen.sh

# Terminal 2: Run autonomous SLAM
./scripts/run_autonomous_slam.sh
# Robot explores automatically using frontier detection
```

### Manual Robot Control
```bash
# Start teleop control (requires Gazebo + robot spawned)
./scripts/run_teleop.sh
```

### Emergency Cleanup
```bash
# Graceful cleanup of all ROS processes
./scripts/kill_all_ros.sh

# Force kill everything (when things are stuck)
./scripts/kill_ros_force.sh

# Diagnose issues
./scripts/diagnose_ros.sh
```

## System Requirements

- ROS2 (Humble or later)
- TurtleBot3 packages
- Gazebo simulation
- Python 3 (for maze generation)
- Built warehouse_robot_system package

## Features

### Autonomous SLAM System
- Fully autonomous frontier-based exploration
- High-level state machine (MAPPING → OPERATIONAL)
- A* pathfinding with obstacle avoidance
- Returns to origin when mapping complete
- Cartographer SLAM for real-time mapping
- RViz visualization for monitoring

### Warehouse Robot System
- Polymorphic robot design with factory pattern
- Integration with autonomous SLAM
- Factory pattern for robot creation

## Troubleshooting

### Build Issues
```bash
# Use the build script for Anaconda conflicts
./scripts/build_project.sh clean

# Or manually in workspace
cd turtlebot3_ws
colcon build --packages-select warehouse_robot_system
```

### Missing Dependencies
```bash
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-cartographer*
sudo apt install libcurl4-openssl-dev
```

### Anaconda/Conda Conflicts
```bash
# Deactivate conda environment
conda deactivate

# Use the build script that handles conflicts
./scripts/build_project.sh
```

### Stuck Processes
```bash
# When ROS nodes won't stop normally
./scripts/kill_all_ros.sh

# When everything is completely stuck
./scripts/kill_ros_force.sh
```

### Gazebo Model Issues
```bash
# Set environment variables
export TURTLEBOT3_MODEL=burger
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models
```