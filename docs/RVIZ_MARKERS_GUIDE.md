# RViz Damage Markers Setup Guide

## Overview
The inspection robot publishes detected damage sites as visual markers in RViz. You need to add the MarkerArray display to see them.

## Quick Setup

### 1. Add MarkerArray Display in RViz

1. **Open RViz** (should already be running with your system)

2. **Click "Add" button** (bottom left of Displays panel)

3. **Select "By topic" tab**

4. **Find and expand** `/inspection/damage_markers`

5. **Select "MarkerArray"** and click **OK**

### 2. Verify Display Settings

The MarkerArray display should now appear in your Displays panel. Check these settings:

- **Topic**: `/inspection/damage_markers`
- **Enabled**: ✓ (checked)
- **Marker Topic**: Should show the topic is active

### 3. What You'll See

When AprilTags are detected during patrol, you'll see:

#### Green Cylinders
- **Color**: Green (RGB: 0, 1, 0)
- **Size**: 0.3m diameter, 1.0m height
- **Position**: At the detected damage location
- **Meaning**: AprilTag detected and saved

#### White Text Labels
- **Position**: Above each cylinder (1.2m height)
- **Content**: "ID: X" where X is the AprilTag ID
- **Size**: 0.3m text height

#### Yellow Cylinders (if any)
- **Color**: Yellow (RGB: 1, 1, 0)
- **Meaning**: Manually marked sites without AprilTag detection

## Marker Details

### Published Topic
```
Topic: /inspection/damage_markers
Type: visualization_msgs/msg/MarkerArray
Frame: map
```

### Marker Properties
```cpp
// Cylinder marker
- Namespace: "damage_sites"
- Type: CYLINDER
- Scale: 0.3m x 0.3m x 1.0m
- Color: Green (0, 1, 0, 0.8) for detected tags
- Lifetime: Persistent (0 = forever)

// Text marker
- Namespace: "damage_labels"
- Type: TEXT_VIEW_FACING
- Scale: 0.3m height
- Color: White (1, 1, 1, 1)
- Text: "ID: <tag_id>"
```

## Troubleshooting

### Markers Not Showing

1. **Check if topic is publishing**:
   ```bash
   ros2 topic echo /inspection/damage_markers --once
   ```

2. **Check if MarkerArray display is enabled**:
   - Look in RViz Displays panel
   - Ensure checkbox is checked

3. **Check frame**:
   - Markers are in "map" frame
   - Ensure your Fixed Frame in RViz is set to "map"

4. **Check if any sites are detected**:
   ```bash
   cat turtlebot3_ws/damage_sites.yaml
   ```

### Markers Appear But Are Wrong Color

- Green = AprilTag detected (apriltag_id >= 0)
- Yellow = Manual site without tag (apriltag_id < 0)

### Markers Too Small/Large

Edit the constants in `InspectionRobot.cpp` line ~1555:
```cpp
marker.scale.x = 0.3;  // Diameter (increase for larger)
marker.scale.y = 0.3;
marker.scale.z = 1.0;  // Height
```

Then rebuild:
```bash
cd turtlebot3_ws
colcon build --packages-select warehouse_robot_system
```

## Current Detection Log

Your system has detected:
```
2025-11-03 00:33:05, Damage_1, Tag ID: 2, Position: (0.000, 0.000)
2025-11-03 01:33:25, Damage_2, Tag ID: 2, Position: (1.753, -0.305)
```

These should appear as:
- Two green cylinders on the map
- Text labels showing "ID: 2" above each

## Slower Rotation for Better Detection

The 360° patrol rotation has been slowed down for better AprilTag detection:

### Previous Settings
- Duration: 8 seconds
- Speed: 1.57 rad/s (~90°/s)
- Result: Fast but might miss tags

### New Settings (Applied)
- Duration: 12 seconds  
- Speed: 0.52 rad/s (~30°/s)
- Result: Slower, more thorough scanning

### To Rebuild with New Settings

```bash
cd ~/MTRX3760_Project_2_Fixing/turtlebot3_ws
colcon build --packages-select warehouse_robot_system
source install/setup.bash
```

Then restart your inspection exploration.

## Verification

After adding the MarkerArray display, you should see:

1. ✅ Green cylinders at (0.000, 0.000) and (1.753, -0.305)
2. ✅ White text "ID: 2" above each cylinder
3. ✅ Markers persist even when robot moves away
4. ✅ New markers appear as robot discovers more AprilTags

## Save RViz Configuration

To avoid re-adding the display every time:

1. **File → Save Config As...**
2. Save to: `~/MTRX3760_Project_2_Fixing/turtlebot3_ws/inspection_rviz.rviz`
3. Next time, load this config instead of default

## Alternative: Command Line Check

If you want to verify markers without RViz:

```bash
# See marker data
ros2 topic echo /inspection/damage_markers

# Count markers
ros2 topic echo /inspection/damage_markers --once | grep "id:" | wc -l
```

## Summary

1. **Add MarkerArray display** in RViz for `/inspection/damage_markers`
2. **Rebuild workspace** to apply slower rotation speed
3. **Restart inspection** to see improved detection
4. **Save RViz config** for future use

The markers will now appear automatically as the robot detects AprilTags during patrol!
