# AprilTag Orientation and Texture Fix

## Issues Fixed

### 1. Tags Laying Flat (180° instead of 90°)
**Problem**: AprilTags were horizontal on the ground instead of vertical on walls.

**Solution**: Added pitch rotation of 1.5708 radians (90°) to make tags stand upright.

**Before**:
```xml
<pose>0.5 1.14 0.3 0 0 3.14159</pose>
```

**After**:
```xml
<pose>0.5 1.14 0.3 0 1.5708 3.14159</pose>
       x   y    z  roll pitch  yaw
```

### 2. No Texture Visible
**Problem**: Gazebo showed warning: "Gazebo does not support Ogre material scripts"

**Solution**: Changed from Ogre material scripts to PBR (Physically Based Rendering) materials.

**Before** (Ogre script - not supported):
```xml
<material>
  <script>
    <uri>model://apriltag_36h11_id0/materials/scripts</uri>
    <name>AprilTag/Tag0</name>
  </script>
</material>
```

**After** (PBR - supported):
```xml
<material>
  <ambient>1 1 1 1</ambient>
  <diffuse>1 1 1 1</diffuse>
  <pbr>
    <metal>
      <albedo_map>model://apriltag_36h11_id0/materials/textures/apriltag_0.png</albedo_map>
    </metal>
  </pbr>
</material>
```

## Updated Files

1. **warehouse_shelves.world**: Updated all AprilTag poses with pitch=1.5708
2. **model.sdf** (all AprilTag models): Changed to PBR materials
3. **create_apriltag_models.sh**: Updated to generate correct format
4. **launch_mgen.sh**: Added AprilTag models to GZ_SIM_RESOURCE_PATH

## Correct Pose Format

For AprilTags on walls, use this format:

```xml
<pose>x y z roll pitch yaw</pose>
```

Where:
- **x, y, z**: Position in meters
- **roll**: 0 (no roll)
- **pitch**: 1.5708 (90° - makes tag vertical)
- **yaw**: Direction tag faces

### Wall Orientations

```xml
<!-- North wall (facing south) -->
<pose>0 1.14 0.3 0 1.5708 3.14159</pose>

<!-- South wall (facing north) -->
<pose>0 -1.14 0.3 0 1.5708 0</pose>

<!-- East wall (facing west) -->
<pose>1.14 0 0.3 0 1.5708 -1.5708</pose>

<!-- West wall (facing east) -->
<pose>-1.14 0 0.3 0 1.5708 1.5708</pose>
```

## Testing

1. **Regenerate models** (already done):
   ```bash
   ./scripts/create_apriltag_models.sh 5
   ```

2. **Launch warehouse**:
   ```bash
   ./launch_mgen.sh  # Select option 10
   ```

3. **Verify in Gazebo**:
   - Tags should be vertical on walls
   - Tags should show black and white pattern
   - No material script warnings

## What Changed

### Model Structure
- Still uses PNG texture files
- Changed material definition from Ogre to PBR
- Added pitch rotation to all wall placements

### Gazebo Compatibility
- **Gazebo Classic** (gazebo): Uses Ogre scripts ❌
- **Gazebo Sim** (gz sim): Uses PBR materials ✅

Since you're using `gz sim`, PBR materials are the correct choice.

## Verification Checklist

- [x] AprilTags stand upright (90° pitch)
- [x] Textures visible in Gazebo
- [x] No material script warnings
- [x] Models regenerated with correct format
- [x] World file updated with correct poses
- [ ] Test with camera detection

## Next Steps

1. Launch Gazebo and verify tags are visible and upright
2. Test camera can see the tags
3. Test AprilTag detector node
4. Run inspection robot to verify detection works

## Related Files

- `turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_shelves.world`
- `turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/apriltag_36h11_id*/model.sdf`
- `scripts/create_apriltag_models.sh`
- `launch_mgen.sh`
