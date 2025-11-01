# Zone Definition Mode

## Overview

Added an interactive zone definition mode to the system menu, allowing users to visually define and save delivery zones before starting delivery operations.

## Features

### 1. New Menu Option

The mode selection menu now includes:

```
[1] 📍 DEFINE DELIVERY ZONES
    • Click points in RViz to mark zones
    • Visualize zones on map
    • Save zones for delivery mode
    • Edit existing zones

[2] 📦 DELIVERY MODE
    • Multi-point delivery operations
    • Uses saved delivery zones
    • Route optimization (TSP)
    • Delivery logging to CSV

[3] 🔍 INSPECTION MODE (Coming Soon)
[4] ❌ EXIT
```

### 2. Zone Marker Node

New `zone_marker_node` that:
- Subscribes to `/clicked_point` from RViz
- Publishes visual markers to `/delivery_zones/markers`
- Automatically saves zones to `delivery_zones.yaml`
- Loads existing zones on startup
- Updates markers in real-time

### 3. Visual Markers

Each delivery zone is visualized with:
- **Cylinder marker**: 30cm diameter, colored (cycles through RGB)
- **Text label**: Zone name displayed above cylinder
- **Persistent**: Markers stay visible until node stops
- **Color-coded**: Different colors for each zone

### 4. Increased Relocalization

Relocalization duration increased for better accuracy:
- **Duration**: 8 seconds (2 full rotations)
- **Speed**: 1.57 rad/s (~90°/second)
- **Purpose**: Better SLAM localization, especially in localization mode

## Usage

### Step 1: Define Zones

```bash
./scripts/run_autonomous_slam.sh -preload
# Select option 1: DEFINE DELIVERY ZONES
```

1. RViz opens with the map
2. Select "Publish Point" tool from toolbar
3. Click on accessible areas of the map
4. Each click creates a new zone with a colored marker
5. Zones are automatically saved
6. Press Ctrl+C when done

### Step 2: Run Deliveries

```bash
# After defining zones, select option 2: DELIVERY MODE
```

The delivery robot will:
1. Load the saved zones
2. Perform relocalization spin (8 seconds)
3. Execute deliveries using the defined zones
4. Return home when complete

## Zone File Format

Zones are saved to `delivery_zones.yaml`:

```yaml
delivery_zones:
- name: Zone_1
  x: 0.5
  y: -0.91
  description: Delivery zone added via RViz
- name: Zone_2
  x: -1.18
  y: -0.58
  description: Delivery zone added via RViz
```

## Visualization

### In RViz

Zones appear as:
- Colored cylinders at zone positions
- Text labels showing zone names
- Height: 30cm above ground
- Diameter: 30cm
- Colors: Red, Green, Blue (cycling)

### Marker Topics

- `/delivery_zones/markers` - MarkerArray with all zone visualizations
- Markers are republished every second for persistence

## Benefits

1. **Visual Feedback**: See exactly where zones are placed
2. **Interactive**: Click and place zones directly on the map
3. **Persistent**: Zones saved automatically
4. **Editable**: Run mode again to add more zones
5. **Clear Workflow**: Separate zone definition from delivery execution
6. **No Command Line**: No need to use separate scripts

## Technical Details

### Zone Marker Node

**Subscriptions:**
- `/clicked_point` (geometry_msgs/PointStamped)

**Publications:**
- `/delivery_zones/markers` (visualization_msgs/MarkerArray)

**Files:**
- Reads/writes: `delivery_zones.yaml`

**Marker Types:**
- CYLINDER: Physical zone representation
- TEXT_VIEW_FACING: Zone name label

### Color Scheme

Zones cycle through colors:
- Zone 1, 4, 7...: Red (1.0, 0.3, 0.3)
- Zone 2, 5, 8...: Green (0.3, 1.0, 0.3)
- Zone 3, 6, 9...: Blue (0.3, 0.3, 1.0)

### Marker Properties

```cpp
// Cylinder
scale.x = 0.3;  // Diameter
scale.y = 0.3;
scale.z = 0.3;  // Height
position.z = 0.15;  // Half height above ground
color.a = 0.7;  // Semi-transparent

// Text
scale.z = 0.15;  // Text height
position.z = 0.5;  // Above cylinder
color.a = 1.0;  // Fully opaque
```

## Comparison: Old vs New Workflow

### Old Workflow
```
1. Start delivery mode
2. Click points in RViz (no visual feedback)
3. Run: ./scripts/delivery_commands.sh save
4. Run: ./scripts/delivery_commands.sh start
5. Hope zones were saved correctly
```

### New Workflow
```
1. Select "Define Delivery Zones" from menu
2. Click points in RViz (see colored markers appear)
3. Press Ctrl+C when done (auto-saves)
4. Select "Delivery Mode" from menu
5. Robot uses saved zones automatically
```

## Future Enhancements

Possible additions:
- Delete zones by clicking near them
- Edit zone names
- Reorder zones
- Import/export zone sets
- Zone validation (check if accessible)
- Distance measurements between zones
- Zone grouping/categories

## Troubleshooting

**Markers not appearing:**
- Check RViz has MarkerArray display for `/delivery_zones/markers`
- Ensure zone_marker_node is running
- Check RViz fixed frame is set to "map"

**Zones not saving:**
- Check write permissions in workspace directory
- Verify YAML file is created: `ls delivery_zones.yaml`
- Check node logs for errors

**Wrong zone positions:**
- Ensure you're clicking on the map, not empty space
- Check "Publish Point" tool is selected
- Verify map is loaded correctly in RViz
