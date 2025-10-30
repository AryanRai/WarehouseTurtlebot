# Viewing Home Position in RViz

## Overview

The home position (0, 0) is now visible in RViz as a TF frame called `home_position`. This helps you see where the robot will return to when exploration is complete or when manually triggered.

## What You'll See

In RViz, you should see:
- **Map frame** (map origin) - The SLAM coordinate system
- **Home position frame** (at 0, 0) - A set of colored axes showing where home is
  - Red axis = X direction
  - Green axis = Y direction  
  - Blue axis = Z direction (up)

## Setup (Already Configured)

The RViz configuration at `config/slam_toolbox.rviz` has been updated with:

1. **Fixed Frame changed to `map`** - Required for SLAM visualization
2. **TF Display added** - Shows coordinate frames
3. **Home position enabled** - The `home_position` frame is visible by default

## Viewing in RViz

When you run the autonomous SLAM system:

```bash
./run_slam_no_conda.sh
```

RViz will open automatically with the home position visible.

### If You Don't See It

1. **Check TF display is enabled:**
   - In the Displays panel (left side)
   - Look for "TF" in the list
   - Make sure the checkbox next to it is checked

2. **Check home_position frame is enabled:**
   - Expand the "TF" display
   - Look for "Frames" section
   - Find "home_position" and ensure it's checked

3. **Check the static TF broadcaster is running:**
   ```bash
   ros2 topic echo /tf_static --once
   ```
   You should see a transform from `map` to `home_position`

4. **Verify Fixed Frame is set to `map`:**
   - In Global Options (top of Displays panel)
   - Fixed Frame should be "map"
   - If it shows an error, wait for SLAM to initialize (10-20 seconds)

## Manual Configuration (If Needed)

If you need to manually add the TF display:

1. Click "Add" button at bottom of Displays panel
2. Select "By display type" tab
3. Find and select "TF"
4. Click "OK"
5. In the TF display properties:
   - Expand "Frames"
   - Check "home_position"
   - Adjust "Marker Scale" if axes are too small/large (default: 0.5)

## Customization

### Change Marker Size

To make the home position axes larger or smaller:

1. Select the TF display
2. Find "Marker Scale" property
3. Adjust value (0.5 = 50cm axes, 1.0 = 1m axes)

### Show All Frames

To see all TF frames (robot links, etc.):

1. Expand TF display
2. Expand "Frames"
3. Click "All Enabled" checkbox

### Change Colors

The TF axes colors are standard:
- Red = X axis (forward)
- Green = Y axis (left)
- Blue = Z axis (up)

These cannot be changed as they follow the ROS convention.

## Troubleshooting

### "Fixed Frame [map] does not exist"

**Cause:** SLAM Toolbox hasn't started publishing the map frame yet.

**Solution:** Wait 10-20 seconds for SLAM to initialize. The error will disappear once SLAM starts.

### Home position not visible

**Cause:** Static TF broadcaster may not be running.

**Solution:** Check if the autonomous_slam.launch.py is being used:
```bash
ros2 node list | grep static_transform_publisher
```

You should see a node called `/home_position_broadcaster`.

### TF display shows error

**Cause:** TF tree not connected or frames missing.

**Solution:** 
```bash
# View TF tree
ros2 run tf2_tools view_frames

# This creates frames.pdf showing the TF tree
# Check that map -> home_position exists
```

## Technical Details

The home position is published by a static transform broadcaster in the launch file:

```python
Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='home_position_broadcaster',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'home_position']
)
```

This creates a transform:
- **Parent frame:** `map`
- **Child frame:** `home_position`
- **Position:** (0, 0, 0)
- **Orientation:** (0, 0, 0) - no rotation

The transform is "static" meaning it never changes - home is always at the map origin.
