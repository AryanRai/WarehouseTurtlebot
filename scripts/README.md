# MTRX3760 Project 2 - Scripts

This folder contains utility scripts for running the warehouse robot system with SLAM capabilities.

## Available Scripts

### 🏭 Warehouse Robot System Testing
- **`test_warehouse_system.sh`** - Test the warehouse robot system components
  - Tests SLAM module functionality
  - Demonstrates polymorphic robot behavior
  - Validates build and basic operations

### 🤖 SLAM Simulation
- **`run_slam_sim.sh`** - Complete SLAM simulation with RViz
  - Launches RViz for visualization
  - Spawns TurtleBot3 in simulation
  - Starts SLAM (Cartographer) mapping
  - Runs warehouse robot system
  - Provides real-time status monitoring

### 🎮 Robot Control
- **`run_teleop.sh`** - Manual robot control via keyboard
  - Simple teleop interface for TurtleBot3
  - Use w/x for linear velocity, a/d for angular velocity
  - Space/s for emergency stop

### 🗺️ Full Demo
- **`run_full_slam_demo.sh`** - Complete maze + SLAM demonstration
  - Generates custom maze using Mgen
  - Launches Gazebo with the maze
  - Provides instructions for running SLAM simulation
  - Multiple difficulty levels available

## Usage Examples

### Quick Test
```bash
# Test the warehouse robot system
./scripts/test_warehouse_system.sh
```

### Full SLAM Demo
```bash
# Terminal 1: Generate maze and launch Gazebo
./scripts/run_full_slam_demo.sh

# Terminal 2: Start SLAM simulation
./scripts/run_slam_sim.sh

# Terminal 3: Control the robot
./scripts/run_teleop.sh
```

### Manual Maze + SLAM
```bash
# Terminal 1: Generate and launch maze
./launch_mgen.sh

# Terminal 2: Run SLAM simulation
./scripts/run_slam_sim.sh

# Terminal 3: Control robot
./scripts/run_teleop.sh
```

## System Requirements

- ROS2 (Humble or later)
- TurtleBot3 packages
- Gazebo simulation
- Python 3 (for maze generation)
- Built warehouse_robot_system package

## Features Demonstrated

### Polymorphic Robot System
- Factory pattern for robot creation
- Abstract base class with virtual methods
- Specialized InspectionRobot and DeliveryRobot classes
- Dynamic robot type switching capabilities

### SLAM Integration
- Frontier-based exploration using Expanding Wavefront Detection
- A* pathfinding with cost maps
- Grid-based occupancy map processing
- Real-time coordinate transformations

### ROS2 Integration
- Proper message passing and topic communication
- Integration with TurtleBot3 hardware interface
- Cartographer SLAM for real-time mapping
- RViz visualization for monitoring

## Troubleshooting

### Build Issues
```bash
cd turtlebot3_ws
colcon build --packages-select warehouse_robot_system
```

### Missing Dependencies
```bash
# Install TurtleBot3 packages
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-cartographer*
```

### Maze Generator Issues
```bash
# Initialize submodule
git submodule update --init --recursive
```

### Gazebo Model Issues
```bash
# Set environment variables
export TURTLEBOT3_MODEL=burger
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/models
```