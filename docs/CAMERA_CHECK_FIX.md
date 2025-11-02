# Camera Check and TF Stability Fix

## Problem
The inspection exploration mode was hanging at two points:

1. **Camera Check Hanging**: The script showed "Waiting for camera..." repeatedly even though `/camera/image_raw` topic was publishing
2. **TF Check Blocking**: After starting camera nodes, the script would hang indefinitely waiting for TF transforms

## Root Causes

### Camera Check Issue
- Used `ros2 topic hz /camera/image_raw --once` which can fail even when topic exists
- No proper timeout or fallback mechanism
- Didn't verify publisher count before checking messages

### TF Check Issue  
- Used `ros2 run tf2_ros tf2_echo map base_footprint` without timeout
- This command blocks indefinitely if transform doesn't exist
- Caused script to hang completely

## Fixes Applied

### 1. Improved Camera Detection
**Location**: `scripts/run_autonomous_slam.sh` (lines ~875 and ~1395)

**Old approach**:
```bash
if timeout 2s ros2 topic hz /camera/image_raw --once > /dev/null 2>&1; then
```

**New approach**:
```bash
# Check if topic exists and has at least one publisher
if ros2 topic info /camera/image_raw 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    # Try to get one message to verify it's actually publishing
    if timeout 3s ros2 topic echo /camera/image_raw --once > /dev/null 2>&1; then
```

**Benefits**:
- First checks if publishers exist (fast check)
- Then verifies actual message delivery
- Increased timeout from 2s to 3s for reliability
- More robust detection logic

### 2. Fixed TF Transform Checks (Using SLAM Toolbox Method)
**Location**: `scripts/run_autonomous_slam.sh` (lines ~1015 and ~1455)

**Old approach**:
```bash
if ros2 run tf2_ros tf2_echo map base_footprint > /dev/null 2>&1; then
```

**New approach** (improved with timeout):
```bash
if timeout 2s bash -c "ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q 'At time'" 2>/dev/null; then
```

**Benefits**:
- Uses the SAME method as SLAM Toolbox check but with timeout wrapper
- Checks if transform is actually publishing data (not just if command succeeds)
- `grep -q "At time"` verifies real transform data is being output
- **Timeout wrapper** prevents hanging if transform doesn't exist
- Reduced from 5 to 2 consecutive checks (faster startup)
- Progress feedback every 3 attempts
- Maximum 15 attempts (~30 seconds total)
- Prevents the "white robot" issue in RViz

## Testing

To verify the fixes work:

1. **Start your system** (Gazebo + TurtleBot)
2. **Run inspection exploration**:
   ```bash
   ./scripts/run_autonomous_slam.sh
   ```
3. **Expected behavior**:
   - Camera check should complete within 10 seconds
   - TF check should complete within 15 seconds
   - Script should proceed to start inspection robot
   - No indefinite hanging

## What to Watch For

### Camera Check
```
📹 Checking camera feed...
   Waiting for camera... (1/10)
   ✅ Camera feed detected and stable
```

### TF Check
```
🔄 Waiting for TF transforms to stabilize...
   Waiting for TF transforms... (3/15)
   Checking TF stability... (1/2)
   Checking TF stability... (2/2)
   ✅ TF transforms stable (map → base_footprint)
```

This now uses the same reliable method as the SLAM Toolbox initialization check, but with:
- Timeout wrapper to prevent hanging
- Progress feedback every 3 attempts
- Only 2 consecutive checks needed (faster)

## If Issues Persist

### Camera Still Not Detected
1. Verify camera is actually publishing:
   ```bash
   ros2 topic list | grep camera
   ros2 topic info /camera/image_raw
   ros2 topic echo /camera/image_raw --once
   ```

2. Check TurtleBot connection:
   ```bash
   ssh ubuntu@<TURTLEBOT_IP>
   ros2 launch turtlebot3_bringup robot.launch.py
   ```

### TF Still Hanging
1. Check if SLAM Toolbox is running:
   ```bash
   ros2 node list | grep slam
   ```

2. Verify map is loaded:
   ```bash
   ros2 topic echo /map --once
   ```

3. Check TF tree manually:
   ```bash
   ros2 run tf2_tools view_frames
   ```

## Why This Approach Works

The key insight is that the SLAM Toolbox initialization already had a working TF check:

```bash
# From SLAM Toolbox startup (line ~1977)
if ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "At time" 2>/dev/null; then
    MAP_FRAME_READY=true
    echo "   ✅ SLAM Toolbox TF frames ready (map → odom)"
    break
fi
```

This method:
1. Runs `tf2_echo` and captures both stdout and stderr (`2>&1`)
2. Looks for "At time" in the output, which appears when transforms are actually publishing
3. `grep` exits once it finds a match or the pipe closes
4. Is more reliable than checking exit codes alone

**However**, we discovered that without a timeout, this can still hang if the transform never appears. So we improved it:

```bash
# Improved version with timeout wrapper
if timeout 2s bash -c "ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q 'At time'" 2>/dev/null; then
```

The `timeout 2s bash -c` wrapper ensures:
- Each check attempt completes within 2 seconds
- No indefinite hanging if transform doesn't exist
- Still uses the reliable "At time" detection method
- Progress feedback shows user what's happening

The inspection mode now uses this improved approach for checking `map → base_footprint` transforms.

## Summary

These fixes ensure the inspection exploration mode doesn't hang during startup by:
- Using more reliable camera detection with publisher verification
- Using the proven SLAM Toolbox TF check method (grep for "At time")
- Reducing wait times (3 checks instead of 5, faster startup)
- Providing better feedback during checks
- Allowing graceful continuation even if checks fail

The script will now proceed through startup reliably, just like the SLAM Toolbox initialization does, and the robot should appear correctly in RViz (not white).
