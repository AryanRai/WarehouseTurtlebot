# AprilTag Setup for Gazebo Simulation

## Overview

This guide explains how to add AprilTag markers to your Gazebo warehouse world for testing the inspection robot's damage detection capabilities.

## Quick Start

### 1. Generate AprilTag Models

Create multiple AprilTag models (IDs 0-4):

```bash
./scripts/create_apriltag_models.sh 5
```

This creates 5 AprilTag models in:
```
turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/
├── apriltag_36h11_id0/
├── apriltag_36h11_id1/
├── apriltag_36h11_id2/
├── apriltag_36h11_id3/
└── apriltag_36h11_id4/
```

### 2. Launch Warehouse with AprilTags

```bash
./launch_mgen.sh
# Select option 10 (Warehouse with Shelves)
```

The warehouse world already includes 3 AprilTags (ID 0) on different walls.

### 3. Test AprilTag Detection

In a new terminal:
```bash
# Start the inspection robot system
./scripts/run_autonomous_slam.sh
# Select option [5] Inspection Mode
```

## AprilTag Model Structure

Each AprilTag model consists of:

```
apriltag_36h11_id0/
├── model.config          # Model metadata
├── model.sdf             # Model geometry and physics
└── materials/
    ├── scripts/
    │   └── apriltag.material  # Gazebo material definition
    └── textures/
        └── apriltag_0.png     # AprilTag image texture
```

### Model Specifications

- **Size**: 162mm x 162mm (0.162m) - standard AprilTag size
- **Thickness**: 1mm (very thin, like a sticker)
- **Family**: 36h11 (most robust for detection)
- **Static**: Yes (doesn't move or fall)

## Adding AprilTags to World Files

### Basic Syntax

```xml
<include>
  <uri>model://apriltag_36h11_id0</uri>
  <name>apriltag_damage_1</name>
  <pose>x y z roll pitch yaw</pose>
</include>
```

### Wall Placement Examples

#### North Wall (facing south)
```xml
<include>
  <uri>model://apriltag_36h11_id0</uri>
  <name>apriltag_north</name>
  <pose>0 1.14 0.3 0 0 3.14159</pose>
</include>
```

#### South Wall (facing north)
```xml
<include>
  <uri>model://apriltag_36h11_id1</uri>
  <name>apriltag_south</name>
  <pose>0 -1.14 0.3 0 0 0</pose>
</include>
```

#### East Wall (facing west)
```xml
<include>
  <uri>model://apriltag_36h11_id2</uri>
  <name>apriltag_east</name>
  <pose>1.14 0 0.3 0 0 -1.5708</pose>
</include>
```

#### West Wall (facing east)
```xml
<include>
  <uri>model://apriltag_36h11_id3</uri>
  <name>apriltag_west</name>
  <pose>-1.14 0 0.3 0 0 1.5708</pose>
</include>
```

### Pose Parameters

- **x, y, z**: Position in meters (z=0.3 is 30cm above ground)
- **roll, pitch, yaw**: Rotation in radians
  - `0` = facing north
  - `1.5708` (π/2) = facing east
  - `3.14159` (π) = facing south
  - `-1.5708` (-π/2) = facing west

### Height Recommendations

- **Ground level**: z = 0.1 (10cm)
- **Robot camera height**: z = 0.3 (30cm) - **RECOMMENDED**
- **High on wall**: z = 0.5 (50cm)

## Creating Custom AprilTags

### Generate Single Tag

```bash
python3 scripts/generate_apriltag.py <tag_id> <output_path>
```

Example:
```bash
python3 scripts/generate_apriltag.py 5 tag_5.png
```

### Generate Multiple Tags

```bash
./scripts/create_apriltag_models.sh 10  # Creates IDs 0-9
```

## Current Warehouse Setup

The `warehouse_shelves.world` includes:

1. **North Wall Tag** (ID 0)
   - Position: (0.5, 1.14, 0.3)
   - Facing: South
   - Visible from center aisle

2. **East Wall Tag** (ID 0)
   - Position: (1.14, 0.5, 0.3)
   - Facing: West
   - Visible from right aisle

3. **South Wall Tag** (ID 0)
   - Position: (-0.5, -1.14, 0.3)
   - Facing: North
   - Visible from center aisle

## Testing AprilTag Detection

### 1. Start Gazebo with Warehouse

```bash
./launch_mgen.sh
# Select option 10
```

### 2. Verify Camera is Working

```bash
ros2 topic list | grep camera
ros2 topic echo /camera/image_raw --once
```

### 3. Start AprilTag Detector

```bash
ros2 run warehouse_robot_system apriltag_detector_node
```

### 4. Check Detections

```bash
ros2 topic echo /apriltag_detections
```

You should see detections when the robot's camera can see a tag.

### 5. Test with Inspection Robot

```bash
./scripts/run_autonomous_slam.sh
# Select [5] Inspection Mode
# Mark damage sites near AprilTags
# Start inspections
```

## Troubleshooting

### Tags Not Visible in Gazebo

**Problem**: AprilTags don't appear in simulation

**Solutions**:
1. Check model path is correct:
   ```bash
   ls turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/apriltag_36h11_id0/
   ```

2. Verify texture file exists:
   ```bash
   ls turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/apriltag_36h11_id0/materials/textures/
   ```

3. Check Gazebo resource path:
   ```bash
   echo $GZ_SIM_RESOURCE_PATH
   ```

### Tags Not Detected by Camera

**Problem**: Camera sees tags but detector doesn't detect them

**Solutions**:
1. Check camera topic:
   ```bash
   ros2 topic hz /camera/image_raw
   ```

2. Verify AprilTag detector is running:
   ```bash
   ros2 node list | grep apriltag
   ```

3. Check detection topic:
   ```bash
   ros2 topic echo /apriltag_detections
   ```

4. Ensure tag is in camera FOV and at correct distance (< 2m)

### Wrong Tag ID Detected

**Problem**: Detector reports wrong tag ID

**Solution**: The simplified tag generator creates basic patterns. For production use, install the real apriltag library:

```bash
pip install apriltag
```

Then modify `generate_apriltag.py` to use the real library.

## Advanced: Using Real AprilTag Library

For accurate tag generation:

```python
import apriltag

# Generate real AprilTag
tag_family = apriltag.apriltag("tag36h11")
tag_image = tag_family.create(tag_id=0)
```

## Adding More Tags to Warehouse

Edit `warehouse_shelves.world` and add:

```xml
<!-- Additional AprilTag on shelf -->
<include>
  <uri>model://apriltag_36h11_id4</uri>
  <name>apriltag_shelf_1</name>
  <pose>0 0.38 0.4 0 0 0</pose>
</include>
```

## Best Practices

1. **Tag Placement**:
   - Place at robot camera height (0.3m)
   - Ensure clear line of sight
   - Avoid placing too close to corners
   - Space tags at least 0.5m apart

2. **Tag IDs**:
   - Use unique IDs for each damage site
   - Document which ID corresponds to which location
   - Keep IDs sequential for easier management

3. **Testing**:
   - Test detection from multiple angles
   - Verify detection range (typically 0.5m - 2m)
   - Check lighting conditions in simulation

4. **Performance**:
   - Limit total tags in world to < 20
   - Use appropriate image resolution (512x512 is good)
   - Static tags perform better than dynamic

## Related Documentation

- [Inspection Robot Implementation](INSPECTION_ROBOT_IMPLEMENTATION.md)
- [Assignment Requirements](../Docs/assignment.md)
- [Quick Start Guide](QUICK_START.md)

## References

- [AprilTag Library](https://april.eecs.umich.edu/software/apriltag)
- [Gazebo Models](https://gazebosim.org/docs)
- [ROS 2 Camera Integration](https://docs.ros.org/en/jazzy/Tutorials.html)
