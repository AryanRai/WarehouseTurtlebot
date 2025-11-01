# Inspection Robot Implementation

## Overview

The Inspection Robot is designed for warehouse damage detection using camera and AprilTag markers. It provides two modes of operation:

1. **Inspection Exploration Mode**: Autonomous exploration to discover and mark damage sites
2. **Inspection Mode**: Navigate to pre-defined damage sites, read AprilTag IDs, and log results

## Features

### Core Functionality
- **Camera Integration**: Uses ROS 2 camera topics for visual inspection
- **AprilTag Detection**: Reads AprilTag IDs at damage sites for identification
- **Autonomous Navigation**: A* pathfinding with obstacle avoidance
- **Route Optimization**: TSP-based route planning for efficient inspection
- **Precise Docking**: Accurate positioning at damage sites for tag reading
- **Inspection Logging**: CSV logging of all inspection results

### Architecture
The Inspection Robot reuses the proven delivery robot architecture:
- `InspectionRobot.hpp/cpp`: Main robot controller
- `InspectionStructures.hpp`: Data structures (DamageSite, InspectionRequest, InspectionRecord)
- `inspection_robot_node.cpp`: ROS 2 node wrapper
- `inspection_commands.sh`: Helper script for common operations

## Usage

### 1. Start the System

```bash
./scripts/run_autonomous_slam.sh
```

Select option **[5] 🔍 INSPECTION MODE** from the menu.

### 2. Choose Route Optimization

- **[1] ORDERED MODE**: Sequential site visits (Damage_1 → Damage_2 → ...)
- **[2] OPTIMIZED MODE (TSP)**: Shortest path using A* distance matrix and Simulated Annealing

### 3. Define Damage Sites

In RViz:
1. Use the "Publish Point" tool (top toolbar)
2. Click on the map to mark damage locations
3. Each click creates a new site (Damage_1, Damage_2, etc.)

### 4. Save Sites

```bash
./scripts/inspection_commands.sh save
```

### 5. Start Inspections

```bash
./scripts/inspection_commands.sh start
```

### 6. Monitor Progress

```bash
./scripts/inspection_commands.sh status
./scripts/inspection_commands.sh log
```

## Inspection Commands

The `inspection_commands.sh` script provides:

- `save` - Save current damage sites to file
- `start` - Start inspection operations
- `status` - Show inspection status
- `log` - View inspection log
- `sites` - List all damage sites
- `clear` - Clear all damage sites

## Data Files

### damage_sites.yaml
Stores damage site locations and AprilTag IDs:
```yaml
damage_sites:
  - name: Damage_1
    x: 2.5
    y: 1.3
    apriltag_id: 42
    description: "Wall damage detected"
```

### inspection_log.csv
Logs all inspection results:
```csv
Timestamp,SiteName,AprilTagID,Position_X,Position_Y,Distance(m),Time(s),Status,Notes
2025-11-01 14:30:15,Damage_1,42,2.5,1.3,5.2,12.3,Success,AprilTag detected successfully
```

## AprilTag Detection

### Requirements
- Camera must be publishing to `/camera/image_raw` or `/camera/unified`
- AprilTag detector node must be running
- AprilTags must be visible when robot reaches damage site

### Detection Process
1. Robot navigates to damage site
2. Stops within 8cm of target position
3. Waits up to 5 seconds for AprilTag detection
4. Logs tag ID or timeout
5. Moves to next site

### Timeout Handling
If no AprilTag is detected within 5 seconds:
- Inspection marked as failed
- Logged with "No AprilTag detected (timeout)" note
- Robot continues to next site

## Route Optimization

### Ordered Mode
- Visits sites in defined order
- Fast startup, predictable route
- Good for pre-planned inspection sequences

### TSP Mode
- Builds A* distance matrix between all sites
- Uses Simulated Annealing to find shortest tour
- Minimizes total travel distance
- Best for efficiency

## Integration with Assignment Requirements

### Inspection Robot (Assignment Requirement)
✅ **Has camera + LiDAR + odometry**
✅ **Locates damages in the warehouse** (via AprilTag markers)
✅ **Saves damage records to disk** (inspection_log.csv)

### Key Differences from Delivery Robot
- Uses camera for AprilTag detection
- Reads and logs tag IDs at each site
- Timeout handling for missing tags
- Inspection-specific data structures

## Testing in Gazebo

### Setup AprilTags in Simulation
1. Launch Gazebo with map world:
   ```bash
   ./launch_mgen.sh
   # Select option 10 (map world)
   ```

2. Create AprilTag images and attach to walls in Gazebo
3. Ensure camera is properly configured

### Test Workflow
1. Start inspection mode
2. Define damage sites near AprilTags
3. Save sites
4. Start inspections
5. Monitor tag detection in logs

## Troubleshooting

### No AprilTag Detected
- Check camera is publishing: `ros2 topic echo /camera/image_raw`
- Verify AprilTag detector is running: `ros2 node list | grep apriltag`
- Check tag is visible and within camera FOV
- Increase timeout if needed (edit TAG_DETECTION_TIMEOUT in InspectionRobot.hpp)

### Robot Not Moving
- Check map is loaded: `ros2 topic echo /map --once`
- Verify SLAM Toolbox is in localization mode
- Check for path planning errors in logs

### Sites Not Saving
- Ensure workspace is writable
- Check for YAML syntax errors
- Verify service call succeeded: `./scripts/inspection_commands.sh save`

## Future Enhancements

- Real-time damage classification using computer vision
- Multi-robot inspection coordination
- Automatic AprilTag generation and placement
- Integration with warehouse management system
- Photo capture at damage sites
- Severity assessment and prioritization

## Related Documentation

- [Delivery Robot](../Docs/delivery_robot.md)
- [SLAM Setup](../Docs/slam_toolbox_setup.md)
- [Assignment Requirements](../Docs/assignment.md)
- [Quick Start Guide](docs/QUICK_START.md)
