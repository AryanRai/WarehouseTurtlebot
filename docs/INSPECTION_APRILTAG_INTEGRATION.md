# AprilTag Detection Integration with Inspection Robot

## ✅ Integration Complete

The AprilTag detection system has been successfully integrated with the inspection robot for both exploration and inspection modes.

## 🎯 Features Implemented

### 1. AprilTag Detection Integration
- **Subscription**: Robot subscribes to `/apriltag_detections` topic
- **Quality Filtering**: Uses decision margin ≥ 45 and hamming = 0
- **Temporal Filtering**: Optional 1-second stability requirement (disabled by default)
- **Real-time Detection**: Detects AprilTags during patrol and inspection

### 2. Automatic Site Discovery (Exploration Mode)
- **Systematic Patrol**: Robot patrols accessible areas
- **Auto-Discovery**: Automatically detects and saves AprilTag locations
- **Duplicate Prevention**: Checks if tag already discovered within 1m
- **File Logging**: Saves to `inspection_exploration_log.csv` with:
  - Timestamp
  - Site name
  - AprilTag ID
  - Position (X, Y coordinates)
  - Patrol point index
  - Notes

### 3. RViz Visualization
- **Cylinder Markers**: Visual markers at each damage site
  - 🟢 **Green**: Sites with detected AprilTags
  - 🟡 **Yellow**: Sites without detected tags yet
- **Text Labels**: Shows AprilTag ID or site name above each marker
- **Persistent Display**: Markers remain visible in RViz
- **Topic**: `/inspection/damage_markers`

### 4. File Saving with Coordinates
- **YAML Format**: `damage_sites.yaml` stores:
  ```yaml
  - name: Damage_1
    apriltag_id: 1
    position:
      x: 2.5
      y: 3.2
      z: 0.0
    description: "Automatically discovered during patrol"
    discovered_time: 2025-11-02 14:30:45
  ```
- **CSV Log**: `inspection_exploration_log.csv` for exploration records
- **Inspection Log**: `inspection_log.csv` for inspection results

## 🚀 How to Use

### Exploration Mode (Discover AprilTags)

```bash
# Set environment variable
export INSPECTION_MODE=exploration

# Run the inspection robot
ros2 run warehouse_robot_system inspection_robot_node

# Start exploration
ros2 service call /start_inspections std_srvs/srv/Trigger
```

**What happens:**
1. Robot patrols systematically through accessible areas
2. When AprilTag detected → automatically saves location
3. Markers appear in RViz showing discovered sites
4. Coordinates saved to files

### Inspection Mode (Visit Known Sites)

```bash
# Normal mode (default)
ros2 run warehouse_robot_system inspection_robot_node

# Add sites manually in RViz (click points)
# Or load from file

# Start inspections
ros2 service call /start_inspections std_srvs/srv/Trigger
```

**What happens:**
1. Robot navigates to each damage site
2. Reads AprilTag ID at each location
3. Logs inspection results with coordinates
4. Updates markers in RViz

## 📊 RViz Setup

To visualize the damage site markers:

1. **Add MarkerArray Display**:
   - Click "Add" in RViz
   - Select "MarkerArray"
   - Set topic to `/inspection/damage_markers`

2. **View Markers**:
   - Green cylinders = Sites with detected tags
   - Yellow cylinders = Sites without tags
   - White text = AprilTag ID or site name

## 📁 Output Files

### 1. damage_sites.yaml
```yaml
sites:
  - name: Damage_1
    apriltag_id: 1
    position: {x: 2.5, y: 3.2, z: 0.0}
    description: "Automatically discovered during patrol"
  - name: Damage_2
    apriltag_id: 5
    position: {x: -1.2, y: 4.5, z: 0.0}
    description: "Automatically discovered during patrol"
```

### 2. inspection_exploration_log.csv
```csv
Timestamp,SiteName,AprilTagID,Position_X,Position_Y,PatrolPoint,Notes
2025-11-02 14:30:45,Damage_1,1,2.500,3.200,5,Discovered during systematic patrol
2025-11-02 14:32:10,Damage_2,5,-1.200,4.500,12,Discovered during systematic patrol
```

### 3. inspection_log.csv
```csv
Timestamp,SiteName,AprilTagID,Position_X,Position_Y,Distance(m),Time(s),Status,Notes
2025-11-02 15:00:00,Damage_1,1,2.500,3.200,15.3,45.2,Success,Tag detected
2025-11-02 15:02:30,Damage_2,5,-1.200,4.500,8.7,32.1,Success,Tag detected
```

## 🔧 Integration Details

### Code Changes

**1. InspectionRobot.hpp**
- Added `visualization_msgs` includes
- Added `marker_pub_` publisher
- Added `publishSiteMarkers()` method

**2. InspectionRobot.cpp**
- Implemented `publishSiteMarkers()` - Creates RViz markers
- Updated `addSite()` - Calls `publishSiteMarkers()`
- Updated `saveDiscoveredSite()` - Calls `publishSiteMarkers()`
- Updated constructor - Initializes marker publisher, loads and visualizes sites

**3. CMakeLists.txt**
- Added `visualization_msgs` dependency to `inspection_robot_lib`
- Added `visualization_msgs` dependency to `inspection_robot_node`

### AprilTag Detector Configuration

The inspection robot uses the AprilTag detector with:
- **Decision Margin**: 45.0 (tuned for ID 1 tags)
- **Hamming Distance**: 0 (perfect match only)
- **Temporal Filtering**: Disabled by default (instant detection)
- **Quality Filtering**: Always active

## 🎮 Testing

### Test 1: Exploration Mode
```bash
# Terminal 1: Start simulation
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Start AprilTag detector
ros2 run warehouse_robot_system apriltag_detector_node

# Terminal 3: Start inspection robot in exploration mode
export INSPECTION_MODE=exploration
ros2 run warehouse_robot_system inspection_robot_node

# Terminal 4: Start exploration
ros2 service call /start_inspections std_srvs/srv/Trigger

# Watch RViz for green markers appearing as tags are discovered
```

### Test 2: Inspection Mode
```bash
# After exploration, sites are saved
# Run in normal mode to visit discovered sites
ros2 run warehouse_robot_system inspection_robot_node

# Start inspections
ros2 service call /start_inspections std_srvs/srv/Trigger
```

## 📋 Topics and Services

### Subscribed Topics
- `/camera/image_raw` - Camera feed (via AprilTag detector)
- `/apriltag_detections` - AprilTag detections
- `/clicked_point` - Manual site marking in RViz
- `/map` - Occupancy grid for navigation
- `/odom` - Odometry for localization

### Published Topics
- `/inspection/damage_markers` - RViz visualization markers
- `/inspection/status` - Status messages
- `/cmd_vel` - Velocity commands

### Services
- `/start_inspections` - Start exploration or inspection
- `/save_damage_sites` - Save sites to file

## ✅ Success Criteria

Your integration is working when:
- ✅ AprilTags detected during exploration
- ✅ Sites automatically saved with coordinates
- ✅ Green markers appear in RViz at tag locations
- ✅ Files contain correct coordinates
- ✅ Inspection mode visits saved sites
- ✅ No duplicate sites within 1m

## 🎉 Benefits

1. **Automatic Discovery**: No manual site marking needed
2. **Coordinate Tracking**: All sites saved with precise locations
3. **Visual Feedback**: RViz markers show discovered sites
4. **Persistent Storage**: Sites saved to files for later use
5. **Quality Filtering**: Only reliable detections saved
6. **Duplicate Prevention**: Smart checking prevents redundant entries

## 🔄 Next Steps (Optional)

If you want to add color detection later:
1. The infrastructure is ready
2. Just enable the color detector node
3. Update `saveDiscoveredSite()` to include damage type
4. Modify markers to show damage classification

The system is ready for production use with AprilTag detection!
