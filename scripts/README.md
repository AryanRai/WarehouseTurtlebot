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
- **`run_slam_sim.sh`** - Complete SLAM simulation with RViz
  - Launches RViz for visualization
  - Spawns TurtleBot3 in simulation
  - Starts SLAM (Cartographer) mapping
  - Runs warehouse robot system
  - Provides real-time status monitoring

- **`spawn_robot.sh`** - Spawn TurtleBot3 in existing Gazebo simulation
  - Spawns robot at specified coordinates
  - Starts robot_state_publisher
  - Validates Gazebo is running

### 🎮 Robot Control
- **`run_teleop.sh`** - Manual robot control via keyboard
  - Simple teleop interface for TurtleBot3
  - Use w/x for linear velocity, a/d for angular velocity
  - Space/s for emergency stop

## Usage Examples

### Build the Project
```bash
# Build all packages
./scripts/build_project.sh

# Clean build if needed
./scripts/build_project.sh clean
```

### SLAM Simulation
```bash
# Terminal 1: Generate and launch maze
./launch_mgen.sh

# Terminal 2: Run SLAM simulation
./scripts/run_slam_sim.sh

# Terminal 3: Control robot
./scripts/run_teleop.sh
```

### Manual Robot Control
```bash
# Start teleop control (requires Gazebo + robot spawned)
./scripts/run_teleop.sh
```

## System Requirements

- ROS2 (Humble or later)
- TurtleBot3 packages
- Gazebo simulation
- Python 3 (for maze generation)
- Built warehouse_robot_system package

## Features

### SLAM Integration
- Cartographer SLAM for real-time mapping
- RViz visualization for monitoring
- Manual teleop control (wall following disabled)

### Warehouse Robot System
- Polymorphic robot design with factory pattern
- Frontier-based exploration algorithms
- A* pathfinding implementation

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

### Gazebo Model Issues
```bash
# Set environment variables
export TURTLEBOT3_MODEL=burger
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models
```