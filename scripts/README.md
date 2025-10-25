# Scripts Directory

Utility scripts for running the TurtleBot3 warehouse robot system with autonomous SLAM.

## Quick Start

### Simulation Mode
```bash
# 1. Build the project
./scripts/build_project.sh

# 2. Launch Gazebo with maze
./launch_mgen.sh

# 3. Run autonomous SLAM (in new terminal)
./scripts/run_autonomous_slam.sh
```

### Physical Robot Mode
```bash
# 1. Start hardware on TurtleBot
./scripts/turtlebot_bringup.sh start

# 2. Connect to TurtleBot
source scripts/ros_link_turtlebot.sh

# 3. Run autonomous SLAM
./scripts/run_autonomous_slam.sh
# Answer 'y' when prompted to use physical robot
```

## All Scripts

### 🛠️ Build & Setup
- **`build_project.sh`** - Build project with Anaconda conflict resolution
  ```bash
  ./scripts/build_project.sh        # Normal build
  ./scripts/build_project.sh clean  # Clean build
  ```

### 🤖 Robot Operation

#### Autonomous SLAM
- **`run_autonomous_slam.sh`** - Autonomous mapping with frontier exploration
  - Supports both simulation and physical robot
  - Auto-detects Gazebo or prompts for physical mode
  - Frontier-based exploration with A* pathfinding
  - Returns to origin when complete
  ```bash
  ./scripts/run_autonomous_slam.sh
  ```

#### Manual Control
- **`run_teleop.sh`** - Keyboard teleoperation
  - Controls: w/x (linear), a/d (angular), space/s (stop)
  ```bash
  ./scripts/run_teleop.sh
  ```

### 🔗 Physical Robot

- **`turtlebot_bringup.sh`** - Check hardware bringup status on physical TurtleBot
  ```bash
  ./scripts/turtlebot_bringup.sh status  # Check if bringup is running
  ./scripts/turtlebot_bringup.sh stop    # Stop all bringup processes
  ```
  - Requires `sshpass`: `sudo apt-get install sshpass`
  - Connects to TurtleBot at 10.42.0.1 (password: turtlebot)
  
  **Note:** For starting bringup, SSH manually (see `scripts/turtlebot_manual_bringup.md`):
  ```bash
  ssh ubuntu@10.42.0.1
  export ROS_DOMAIN_ID=29
  ros2 launch turtlebot3_bringup robot.launch.py
  ```

- **`ros_link_turtlebot.sh`** - Configure environment for physical TurtleBot
  ```bash
  source scripts/ros_link_turtlebot.sh
  ```
  - Kills local ROS processes
  - Sets ROS_DOMAIN_ID=29
  - Sets RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  - Lists available topics

### 🛑 Cleanup

- **`kill_all_ros.sh`** - Graceful shutdown of all ROS processes
  ```bash
  ./scripts/kill_all_ros.sh
  ```
  - Stops ROS nodes, Gazebo, RViz, Cartographer
  - Cleans shared memory
  - Shows process summary

- **`kill_ros_force.sh`** - Force kill all ROS processes
  ```bash
  ./scripts/kill_ros_force.sh
  ```
  - Use when normal shutdown fails

### 🔍 Diagnostics

- **`diagnose_ros.sh`** - System diagnostics and troubleshooting
  ```bash
  ./scripts/diagnose_ros.sh
  ```
  - Checks nodes, topics, transforms
  - Validates message rates
  - Provides recommendations

### 🎮 Simulation Utilities

- **`spawn_robot.sh`** - Spawn TurtleBot3 in Gazebo
  ```bash
  ./scripts/spawn_robot.sh [x] [y] [yaw]
  ```
  - Default: origin (0, 0, 0)

- **`check_slam_logs.sh`** - View SLAM system logs
- **`test_slam_navigation.sh`** - Test navigation system

## Environment Configuration

All scripts automatically set:
- `ROS_DOMAIN_ID=29` (for physical robot)
- `TURTLEBOT3_MODEL=burger`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` (for physical robot)

## Common Workflows

### Simulation Testing
```bash
./launch_mgen.sh                    # Terminal 1
./scripts/run_autonomous_slam.sh    # Terminal 2
```

### Physical Robot Operation
```bash
./scripts/turtlebot_bringup.sh start          # Start hardware
source scripts/ros_link_turtlebot.sh          # Connect
./scripts/run_autonomous_slam.sh              # Run SLAM (answer 'y')
```

### Cleanup After Session
```bash
./scripts/kill_all_ros.sh           # Stop everything
./scripts/turtlebot_bringup.sh stop # Stop robot hardware (if used)
```

## Troubleshooting

**Build fails with Anaconda conflicts:**
```bash
./scripts/build_project.sh clean
```

**Can't connect to physical robot:**
```bash
# Check TurtleBot is on and connected
ping 10.42.0.1
# Verify hardware bringup is running
./scripts/turtlebot_bringup.sh status
```

**ROS processes won't stop:**
```bash
./scripts/kill_ros_force.sh
```

**Segmentation fault after cleanup:**
```bash
ros2 daemon stop
ros2 daemon start
```