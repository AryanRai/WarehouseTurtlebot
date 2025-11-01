# Autonomous SLAM Implementation

This implementation provides autonomous mapping and exploration for the TurtleBot3 warehouse robot using frontier-based exploration with SLAM Toolbox.

## Overview

The system is based on the reference implementation from `SLAM_Reference.md` and includes:

1. **Frontier Search** - Expanding Wavefront Frontier Detection algorithm
2. **Path Planning** - A* pathfinding with cost map optimization
3. **Motion Control** - Pure Pursuit path following with obstacle avoidance
4. **SLAM Integration** - SLAM Toolbox for mapping and localization

## Architecture

### Core Components

#### 1. SlamController (`SlamController.hpp/cpp`)
- Manages SLAM Toolbox integration
- Provides access to current map and robot pose
- Handles map saving functionality

#### 2. FrontierSearch (`FrontierSearch.hpp/cpp`)
- Implements Expanding Wavefront Frontier Detection
- Identifies unexplored areas (frontiers) in the map
- Groups frontier cells into frontier regions

#### 3. PathPlanner (`PathPlanner.hpp/cpp`)
- A* pathfinding algorithm
- C-Space calculation for obstacle inflation
- Cost map generation for optimal path selection
- Converts between grid and world coordinates

#### 4. ExplorationPlanner (`ExplorationPlanner.hpp/cpp`)
- Selects best frontier to explore
- Plans paths to frontiers using A*
- Combines A* cost and frontier size for decision making
- Detects when exploration is complete

#### 5. MotionController (`MotionController.hpp/cpp`)
- Pure Pursuit path following algorithm
- Obstacle avoidance using local costmap
- Dynamic speed adjustment based on obstacles
- Lookahead-based steering control

#### 6. AutonomousExplorationRobot (`AutonomousExplorationRobot.hpp/cpp`)
- High-level coordinator for all components
- Main control loop for exploration
- Manages exploration state (start/stop/pause)

## Building

```bash
cd ~/MTRX3760_Project_2/turtlebot3_ws
colcon build --packages-select warehouse_robot_system
source install/setup.bash
```

## Running

### Option 1: Standalone Autonomous SLAM Node

```bash
# Terminal 1: Launch Gazebo with warehouse world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch autonomous SLAM exploration
ros2 launch warehouse_robot_system autonomous_slam.launch.py
```

### Option 2: With Debug Visualization

```bash
ros2 launch warehouse_robot_system autonomous_slam.launch.py debug:=true
```

This will publish additional topics for visualization:
- `/exploration/frontier_cells` - Detected frontier cells
- `/exploration/start` - Start positions for A* search
- `/exploration/goal` - Goal positions for A* search
- `/exploration/cspace` - Configuration space (inflated obstacles)
- `/exploration/cost_map` - Cost map for path planning
- `/motion/lookahead` - Pure pursuit lookahead point
- `/motion/fov_cells` - Field of view for obstacle avoidance
- `/motion/close_wall_cells` - Nearby obstacles

### Visualization in RViz

```bash
rviz2 -d $(ros2 pkg prefix warehouse_robot_system)/share/warehouse_robot_system/config/slam_toolbox.rviz
```

Add these topics to visualize the exploration:
- `/map` - SLAM-generated map
- `/exploration/path` - Planned exploration path
- `/exploration/frontier_cells` - Frontiers (if debug enabled)

## Algorithm Details

### Frontier Detection

The system uses **Expanding Wavefront Frontier Detection**:

1. Start breadth-first search from robot's current position
2. Mark all reachable free cells as visited
3. Identify frontier cells (unknown cells adjacent to free space)
4. Group connected frontier cells into frontier regions
5. Calculate centroid and size for each frontier

Minimum frontier size: 8 cells

### Path Planning

**A* Algorithm** with cost map:

```
cost = distance_cost + COST_MAP_WEIGHT * cost_map_value
priority = cost + heuristic(current, goal)
```

- `COST_MAP_WEIGHT = 1000.0` - Heavily penalizes paths near obstacles
- Cost map calculated by iterative dilation from walls
- Hallway detection to prefer center of corridors

### Frontier Selection

Best frontier chosen by combined cost:

```
total_cost = (A_STAR_COST_WEIGHT * path_cost) + (FRONTIER_SIZE_COST_WEIGHT / frontier_size)
```

- `A_STAR_COST_WEIGHT = 10.0` - Prefer closer frontiers
- `FRONTIER_SIZE_COST_WEIGHT = 1.0` - Prefer larger frontiers
- Evaluates top 8 largest frontiers

### Pure Pursuit Control

**Lookahead-based steering**:

```
radius_of_curvature = lookahead_distance / (2 * sin(alpha))
turn_speed = drive_speed / radius_of_curvature
```

- `LOOKAHEAD_DISTANCE = 0.18 m`
- `MAX_DRIVE_SPEED = 0.1 m/s`
- `MAX_TURN_SPEED = 1.25 rad/s`

### Obstacle Avoidance

Local reactive obstacle avoidance:

1. Scan area around robot (FOV = 200°, distance = 25 cells)
2. Calculate weighted average angle to obstacles
3. Apply steering adjustment away from obstacles
4. Slow down when close to obstacles

```
steering_adjustment = -GAIN * average_angle / wall_count
```

## Parameters

### Exploration Parameters

- `NUM_EXPLORE_FAILS_BEFORE_FINISH = 30` - Attempts before declaring exploration complete
- `MAX_NUM_FRONTIERS_TO_CHECK = 8` - Number of frontiers to evaluate
- `MIN_FRONTIER_SIZE = 8` - Minimum cells for valid frontier

### Motion Parameters

- `LOOKAHEAD_DISTANCE = 0.18 m` - Pure pursuit lookahead
- `MAX_DRIVE_SPEED = 0.1 m/s` - Maximum forward speed
- `MAX_TURN_SPEED = 1.25 rad/s` - Maximum angular velocity
- `DISTANCE_TOLERANCE = 0.1 m` - Goal reached threshold

### Obstacle Avoidance Parameters

- `OBSTACLE_AVOIDANCE_GAIN = 0.3` - Steering adjustment gain
- `FOV = 200°` - Field of view for obstacle detection
- `FOV_DISTANCE = 25 cells` - Scan distance
- `FOV_DEADZONE = 80°` - Ignore obstacles directly ahead/behind

## Map Saving

The system automatically saves the map when exploration is complete:

- Map file: `warehouse_map_complete.pgm` and `warehouse_map_complete.yaml`
- Pose file: `warehouse_map_complete_pose.txt`

Manual save:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

## Troubleshooting

### No map being generated
- Check SLAM Toolbox is running: `ros2 node list | grep slam`
- Verify laser scan topic: `ros2 topic echo /scan`
- Check TF tree: `ros2 run tf2_tools view_frames`

### Robot not moving
- Check velocity commands: `ros2 topic echo /cmd_vel`
- Verify path is being published: `ros2 topic echo /exploration/path`
- Check if exploration is enabled

### No frontiers found
- Map may be fully explored
- Check map topic: `ros2 topic echo /map`
- Verify robot pose is valid

### Robot spinning in place
- Check obstacle avoidance parameters
- Verify cost map is reasonable
- May be stuck in local minimum - manually move robot

## References

- **Frontier Exploration**: Yamauchi, B. (1997). "A frontier-based approach for autonomous exploration"
- **Expanding Wavefront**: Keidar, M. & Kaminka, G. A. (2014). "Efficient frontier detection for robot exploration"
- **Pure Pursuit**: Coulter, R. C. (1992). "Implementation of the Pure Pursuit Path Tracking Algorithm"
- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox

## Credits

Implementation adapted from Kai Nakamura's SLAM reference implementation (SLAM_Reference.md)
