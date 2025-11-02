# Camera Stability Check & Headless Mode

## New Features Added

### 1. Camera Stability Check ✅
Before starting inspection exploration, the script now:
- Checks if `/camera/image_raw` topic is publishing
- Verifies camera feed is stable (10-second timeout)
- Prompts user if camera is unavailable
- Allows user to continue or abort

### 2. Viewfinder Verification ✅
When camera UI is enabled:
- Opens AprilTag detection viewfinder
- Prompts user to confirm it's working properly
- If laggy/frozen, automatically restarts in headless mode
- 15-second timeout (auto-continues if no response)

### 3. Headless Mode Flag ✅
New `-nocamui` flag to disable camera viewfinder:
- Detection logic still runs
- No GUI window opens
- Better for unstable network connections
- Reduces bandwidth usage

## Usage

### Normal Mode (with viewfinder)
```bash
./scripts/run_autonomous_slam.sh
# Select option 5 or 6
# Viewfinder will open
# Confirm it's working when prompted
```

### Headless Mode (no viewfinder)
```bash
./scripts/run_autonomous_slam.sh -nocamui
# Select option 5 or 6
# No viewfinder opens
# Detection still works
```

### Combined Flags
```bash
# Preload map + headless camera
./scripts/run_autonomous_slam.sh -preload -nocamui

# All flags
./scripts/run_autonomous_slam.sh -web -preload -nocamui
```

## What Happens

### Inspection Exploration Mode (Option 5)

**With Camera UI (default):**
```
📹 Checking camera feed...
   Waiting for camera... (1/10)
   ✅ Camera feed detected and stable

   Starting AprilTag detector...
   • Visualization: ENABLED (viewfinder will open)
   ✅ AprilTag detector started

   📺 Camera viewfinder should be visible now
   • Green boxes = Detected AprilTags
   • Tag IDs shown above boxes

   ⚠️  If viewfinder is laggy or frozen:
   1. Press Ctrl+C to stop
   2. Restart with: ./scripts/run_autonomous_slam.sh -nocamui

   Is the viewfinder working properly? (yes/no): yes
   ✅ Viewfinder confirmed working
```

**With -nocamui flag:**
```
📹 Checking camera feed...
   ✅ Camera feed detected and stable

   📷 Camera UI disabled (-nocamui flag)
   Starting AprilTag detector...
   • Visualization: DISABLED (headless mode)
   ✅ AprilTag detector started
```

**If camera unavailable:**
```
📹 Checking camera feed...
   Waiting for camera... (1/10)
   Waiting for camera... (2/10)
   ...
   ❌ Camera feed not available or unstable!
   Camera topic /camera/image_raw is not publishing

   Please check:
   • Is the camera node running?
   • Is the TurtleBot connected?
   • Network connection stable?

   Continue anyway? (yes/no): no
   Returning to menu...
```

**If viewfinder is laggy:**
```
   Is the viewfinder working properly? (yes/no): no
   ⚠️  Viewfinder issues detected
   Stopping AprilTag detector...
   Restarting in headless mode...
   ✅ AprilTag detector restarted (headless)
```

### Inspection Mode (Option 6)

Similar checks but simplified:
```
   Checking camera feed...
   ✅ Camera feed detected

   📷 Camera UI disabled (-nocamui flag)  [if flag used]
   Starting AprilTag detector...
   • Visualization: ENABLED  [or DISABLED if -nocamui]
   ✅ AprilTag detector started
```

## Benefits

### 1. Network Stability
- **Problem**: Camera streaming over network can be unstable
- **Solution**: Check camera before starting, offer headless mode
- **Result**: No wasted time starting exploration with broken camera

### 2. User Control
- **Problem**: Laggy viewfinder can freeze or crash
- **Solution**: User confirms viewfinder works, auto-restart if not
- **Result**: Smooth operation even with network issues

### 3. Headless Operation
- **Problem**: GUI requires X11 forwarding, uses bandwidth
- **Solution**: `-nocamui` flag disables GUI but keeps detection
- **Result**: Works over SSH without -X, faster, more reliable

## Troubleshooting

### Camera Not Detected

**Symptoms:**
```
❌ Camera feed not available or unstable!
```

**Solutions:**
1. Check camera node is running:
   ```bash
   ros2 topic list | grep camera
   ros2 topic hz /camera/image_raw
   ```

2. Start camera node if missing:
   ```bash
   ros2 run warehouse_robot_system camera_node
   ```

3. Check TurtleBot connection:
   ```bash
   ping TURTLEBOT_IP
   ```

### Viewfinder Laggy/Frozen

**Symptoms:**
- Viewfinder window opens but shows frozen image
- High latency, choppy video
- Window not responding

**Solutions:**
1. **Immediate**: Answer "no" when prompted
   - Script will restart in headless mode automatically

2. **Next time**: Use `-nocamui` flag
   ```bash
   ./scripts/run_autonomous_slam.sh -nocamui
   ```

3. **Network**: Improve connection
   - Use wired connection instead of WiFi
   - Reduce network traffic
   - Move closer to router

### Detection Not Working

**Symptoms:**
- No AprilTags detected
- Empty `/apriltag_detections` topic

**Check:**
```bash
# 1. Is detector running?
ps aux | grep apriltag_detector_node

# 2. Is it receiving images?
ros2 topic hz /camera/image_raw

# 3. Check detector logs
tail -f /tmp/apriltag_detector.log

# 4. Test detection manually
ros2 topic echo /apriltag_detections
```

## Parameters

### AprilTag Detector Parameters

The detector is started with:
```bash
# With UI
ros2 run warehouse_robot_system apriltag_detector_node \
    --ros-args -p show_visualization:=true

# Without UI (-nocamui)
ros2 run warehouse_robot_system apriltag_detector_node \
    --ros-args -p show_visualization:=false
```

Both modes use the same detection logic:
- Decision margin ≥ 45
- Hamming distance = 0
- Temporal filtering disabled (instant detection)

## Script Flags Summary

| Flag | Description | Use Case |
|------|-------------|----------|
| `-web` | Enable web dashboard | Remote monitoring |
| `-preload` | Skip exploration, load map | Already have map |
| `-nocamui` | Disable camera viewfinder | Unstable network, SSH without -X |

## Examples

### Scenario 1: First Time Setup
```bash
# Start with UI to verify everything works
./scripts/run_autonomous_slam.sh

# Select option 5 (Inspection Exploration)
# Confirm viewfinder works
# Let robot discover AprilTags
```

### Scenario 2: Unstable Network
```bash
# Use headless mode to avoid GUI issues
./scripts/run_autonomous_slam.sh -nocamui

# Select option 5
# Detection works without viewfinder
# More stable operation
```

### Scenario 3: SSH Without X11
```bash
# Connect without -X flag
ssh ubuntu@turtlebot_ip

# Run in headless mode
./scripts/run_autonomous_slam.sh -nocamui

# No GUI needed, detection still works
```

### Scenario 4: Quick Inspection
```bash
# Already have map, no GUI needed
./scripts/run_autonomous_slam.sh -preload -nocamui

# Select option 6 (Inspection Mode)
# Fast startup, no GUI overhead
```

## Technical Details

### Camera Check Implementation
```bash
CAMERA_CHECK_TIMEOUT=10
for i in $(seq 1 $CAMERA_CHECK_TIMEOUT); do
    if timeout 2s ros2 topic hz /camera/image_raw --once > /dev/null 2>&1; then
        CAMERA_READY=true
        break
    fi
    sleep 1
done
```

### Viewfinder Verification
```bash
echo -n "Is the viewfinder working properly? (yes/no): "
read -r -t 15 viewfinder_ok || viewfinder_ok="yes"

if [[ "$viewfinder_ok" != "yes" ]]; then
    # Restart in headless mode
    kill -TERM $APRILTAG_PID
    ros2 run ... -p show_visualization:=false &
fi
```

### Headless Mode Flag
```bash
if [ "$DISABLE_CAMERA_UI" = true ]; then
    ros2 run ... -p show_visualization:=false
else
    ros2 run ... -p show_visualization:=true
fi
```

## Summary

✅ **Camera stability check** - Verifies camera before starting
✅ **Viewfinder verification** - User confirms GUI works
✅ **Auto-recovery** - Restarts in headless if GUI fails
✅ **Headless mode** - `-nocamui` flag for no GUI
✅ **Better reliability** - Works even with network issues
✅ **User-friendly** - Clear prompts and error messages

The system is now robust against camera/network instability!
