# TF Transform Break Fix (Robot Turns White in RViz)

## Problem
When starting deliveries in preload mode (`-preload` flag), the robot sometimes turns white in RViz and gets stuck. This happens because:

1. **TF tree breaks** - The transform chain from `map` → `odom` → `base_footprint` is interrupted
2. **SLAM Toolbox restart** - The script was restarting SLAM Toolbox even when already in localization mode
3. **Race condition** - Delivery robot starts before SLAM is fully initialized

## Root Cause
When using `-preload` mode:
- SLAM Toolbox starts in **localization mode** at the beginning
- When selecting delivery mode, the script was **restarting SLAM Toolbox again**
- This restart breaks the TF tree temporarily or permanently
- The delivery robot starts before TF is re-established

## Solution

### 1. Skip SLAM Restart in Preload Mode
Added check to prevent restarting SLAM when already in localization mode:

```bash
# Check if we're already in preload mode (SLAM already in localization)
if [ "$PRELOAD_MAP" = true ]; then
    echo "   ✓ Already in localization mode (preload)"
    echo "   ✓ SLAM Toolbox running with loaded map"
    echo "   ✓ Skipping SLAM restart to preserve TF tree"
    echo ""
elif [ ! -f "${MAP_FILE_BASE}.yaml" ]; then
    # No map file, continue in mapping mode
else
    # Restart SLAM from mapping to localization mode
fi
```

### 2. Wait for SLAM to be Ready
Added check to ensure SLAM is publishing before starting delivery robot:

```bash
# Wait for SLAM to be ready (check if /map topic is publishing)
echo "   Waiting for SLAM to be ready..."
timeout 10s bash -c 'until ros2 topic hz /map --once > /dev/null 2>&1; do sleep 0.5; done'
sleep 2
```

### 3. Proper Variable Scope
Ensured `PRELOAD_MAP` variable is accessible in the `offer_mode_selection()` function by keeping it in global scope.

## Testing

### Test 1: Preload Mode (Should NOT restart SLAM)
```bash
./scripts/run_autonomous_slam.sh -preload
# Select option [1] for Delivery Mode
# Should see: "✓ Skipping SLAM restart to preserve TF tree"
# Robot should stay visible (not turn white)
```

### Test 2: Normal Mode (Should restart SLAM)
```bash
./scripts/run_autonomous_slam.sh
# Wait for exploration to complete
# Select option [1] for Delivery Mode
# Should see: "Switching SLAM Toolbox: mapping → localization mode..."
# Robot should remain visible after restart
```

### Test 3: Verify TF Tree
While delivery is running, check TF tree:
```bash
# In another terminal
ros2 run tf2_tools view_frames
# Should show complete chain: map → odom → base_footprint
```

## Symptoms of TF Break

### In RViz:
- Robot model turns **white** or **gray**
- Robot disappears from visualization
- Error in RViz status: "Transform [sender=unknown_publisher]"
- Map is visible but robot is not

### In Terminal:
- Warnings about missing transforms
- "Could not transform" errors
- Delivery robot gets stuck (no movement)

## Quick Fix if TF Breaks

If the robot turns white during delivery:

1. **Check TF tree:**
   ```bash
   ros2 run tf2_tools view_frames
   evince frames.pdf
   ```

2. **Restart in preload mode:**
   ```bash
   # Ctrl+C to stop current session
   ./scripts/run_autonomous_slam.sh -preload
   ```

3. **Manually set initial pose in RViz:**
   - Click "2D Pose Estimate" button
   - Click on robot's approximate location on map
   - Drag to set orientation

## Files Modified
- `scripts/run_autonomous_slam.sh` - Added preload check and SLAM ready wait

## Prevention
- Always use `-preload` flag when map already exists
- Don't manually restart SLAM Toolbox during delivery operations
- Ensure Gazebo is running before starting the script
- Wait for "SLAM Toolbox running" message before starting deliveries

## Related Issues
- See `docs/DELIVERY_COMPLETION_FIX.md` for delivery loop fixes
- See `docs/RETURN_HOME_FEATURES.md` for return home functionality
