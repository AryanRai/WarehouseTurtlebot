# Inspection Exploration Mode - Fixed Implementation

## Problem
The inspection exploration mode (option 5) was using frontier exploration, which doesn't work on already-mapped areas. When the map is complete, there are no frontiers left to explore, so the robot would just run recovery behaviors and return home without discovering any AprilTags.

## Solution
Replaced frontier-based exploration with **systematic patrol** that works on already-mapped areas.

## Changes Made

### 1. Pulled New Camera Code
- Merged latest changes from `enhanced-slam-with-camera` branch
- Integrated advanced color detection with AprilTag detection
- New `ColourDetector` can identify damage types (mould=green, water=blue, blood=red)
- Includes calibration mode for HSV tuning

### 2. Updated Script Logic (`scripts/run_autonomous_slam.sh`)

**Before:**
```bash
# Started regular exploration (frontier-based)
"$SCRIPT_DIR/start_autonomous_slam_clean.sh" "$(pwd)" &
```

**After:**
```bash
# Start inspection robot in exploration mode (systematic patrol)
export INSPECTION_MODE="exploration"
ros2 run warehouse_robot_system inspection_robot_node &
```

**Key Improvements:**
- Checks for existing map before starting
- Starts both AprilTag and Color detectors
- Uses inspection robot with `INSPECTION_MODE=exploration` flag
- Systematic grid-based patrol instead of frontier exploration
- Auto-saves discovered damage sites
- Creates completion marker when done
- Proper cleanup of all detector nodes

### 3. Inspection Robot Exploration Mode

The inspection robot now supports two modes:

**Inspection Mode (default):**
- Visits pre-defined damage sites from `damage_sites.yaml`
- Verifies AprilTags at each site
- Generates inspection reports

**Exploration Mode (new):**
- Generates grid-based patrol points across the map
- Systematically visits all accessible areas
- Detects AprilTags and colors continuously
- Auto-saves discovered sites to `damage_sites.yaml`
- Returns home when patrol complete

### 4. How It Works

```
1. Load existing map (from exploration mode)
2. Generate patrol grid (1m spacing)
3. Filter points to accessible areas only
4. Visit each patrol point sequentially
5. At each point:
   - Pause briefly for detection
   - AprilTag detector checks for tags
   - Color detector identifies damage type
   - Auto-save if new damage found
6. Return home when complete
7. Save results to damage_sites.yaml
```

## Usage

### Step 1: Run Exploration Mode First
```bash
./scripts/run_autonomous_slam.sh
# Select option [1] - Exploration Mode
# Wait for robot to map the warehouse
```

### Step 2: Run Inspection Exploration
```bash
# After exploration completes, select option [5]
# Select [1] to start systematic patrol
```

### Step 3: Monitor Progress
```bash
# Watch AprilTag detections
ros2 topic echo /apriltag_detections

# Watch color/damage detections  
ros2 topic echo /warehouse/damage_reports

# Watch robot status
ros2 topic echo /inspection/status
```

### Step 4: Review Results
```bash
# View discovered sites
./scripts/inspection_commands.sh sites

# Check detection log
cat inspection_exploration_log.csv
```

## Output Files

- **damage_sites.yaml** - Discovered damage locations with AprilTag IDs
- **inspection_exploration_log.csv** - Timestamped detection log
- **/tmp/inspection_exploration.log** - Robot execution log
- **/tmp/apriltag_detector.log** - AprilTag detector log
- **/tmp/colour_detector.log** - Color detector log

## Technical Details

### Patrol Point Generation
```cpp
void InspectionRobot::generatePatrolPoints(const nav_msgs::msg::OccupancyGrid& map) {
    // Grid-based patrol pattern with 1m spacing
    // Filters to free space only
    // Checks robot footprint clearance
    // Returns vector of patrol waypoints
}
```

### AprilTag Auto-Discovery
```cpp
void InspectionRobot::onAprilTagDetection(msg) {
    if (exploration_mode_) {
        // Check if tag already discovered
        // Save new site with current position
        // Log to CSV file
        // Update damage_sites.yaml
    }
}
```

### Color Detection Integration
The new `ColourDetector` analyzes regions around detected AprilTags:
- Samples 4 regions (above, below, left, right)
- Converts to HSV color space
- Classifies damage type by dominant color
- Publishes damage reports with confidence scores

## Benefits

1. **Works on Mapped Areas** - No need for unexplored frontiers
2. **Systematic Coverage** - Grid pattern ensures all areas visited
3. **Auto-Discovery** - No manual marking needed
4. **Multi-Sensor** - Combines AprilTag + Color detection
5. **Persistent Results** - Saves to YAML for later inspection
6. **Resumable** - Can run multiple times to discover more sites

## Next Steps

After inspection exploration completes:

1. **Review Sites**: `./scripts/inspection_commands.sh sites`
2. **Run Inspection**: Select option [6] to verify all discovered sites
3. **Save Map**: Select option [7] to save map with damage locations

## Troubleshooting

**No map available error:**
- Run exploration mode (option 1) first
- Wait for map to be published on `/map` topic

**Robot not moving:**
- Check `/tmp/inspection_exploration.log`
- Verify SLAM Toolbox is in localization mode
- Ensure map has accessible free space

**No AprilTags detected:**
- Check camera: `ros2 topic echo /camera/image_raw`
- Verify AprilTags in Gazebo world
- Check `/tmp/apriltag_detector.log`

**Color detection not working:**
- Check `/tmp/colour_detector.log`
- Verify lighting in simulation
- May need HSV calibration (set `calibration_mode: true`)

## Files Modified

- `scripts/run_autonomous_slam.sh` - Updated option 5 logic
- `turtlebot3_ws/src/.../CMakeLists.txt` - Fixed AprilTag detector build
- `turtlebot3_ws/src/.../src/ExplorationPlanner.cpp` - Merged upstream improvements

## Files Added

- `docs/INSPECTION_EXPLORATION_FIX.md` - This documentation

## Camera Integration

The new camera system from `enhanced-slam-with-camera` branch includes:

- **AprilTagDetector** - 16h5 family detection with visualization
- **ColourDetector** - HSV-based damage type classification
- **ImageProcessor** - Base class for camera processing nodes
- **Camera_Node** - Camera interface and image publishing

All camera nodes are now properly integrated and can run alongside the inspection robot.
