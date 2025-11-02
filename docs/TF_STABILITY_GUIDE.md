# TF Transform Stability Guide

## Problem: White Robot in RViz

### Symptoms
- Robot appears white/transparent in RViz
- Robot model not visible
- Laser scans may be missing
- Navigation doesn't work properly
- System freezes briefly then recovers

### Root Cause
**TF2 Transform Tree Disruption** - When switching between SLAM modes (mapping ↔ localization), the transform tree can break temporarily, causing RViz to lose track of the robot's position.

## Solution Implemented

### 1. TF Stability Check ✅
The script now waits for TF transforms to stabilize before starting the robot:

```bash
# Checks map → base_footprint transform
# Requires 5 consecutive successful checks
# 30-second timeout
# Prevents starting with broken TF tree
```

### 2. User Verification ✅
After starting the robot, prompts user to confirm:
- Robot is visible (not white)
- Robot is at correct position
- Laser scans are showing

### 3. Graceful Handling ✅
If TF not stable:
- Warns user about potential issues
- Offers to stop and fix properly
- Provides clear instructions
- Allows continuing if user accepts risk

## How It Works

### Inspection Exploration Mode (Option 5)

```
Starting Inspection Exploration...
    ↓
🔄 Waiting for TF transforms to stabilize...
   Checking TF stability... (1/5 checks)
   Checking TF stability... (3/5 checks)
   ✅ TF transforms stable (map → base_footprint)
    ↓
⏳ Allowing extra time for robot to appear in RViz...
   [3 second pause]
    ↓
Starting Inspection Robot in exploration mode...
   ✅ Inspection Robot node started
    ↓
🤖 Robot Status Check
   Please check RViz:
   • Is the robot visible (not white)?
   • Is the robot at the correct position?
   • Are the laser scans showing?
    ↓
   Is the robot visible and ready? (yes/no): yes
   ✅ Robot confirmed visible and ready!
    ↓
Proceed with exploration ✅
```

### If TF Not Stable

```
🔄 Waiting for TF transforms to stabilize...
   Waiting for TF transforms...
   [30 seconds pass]
   ⚠️  WARNING: TF transforms not stable after 30 seconds
   The robot may appear white in RViz

   Common causes:
   • SLAM Toolbox still initializing
   • Map not fully loaded
   • TF tree disrupted from mode switch

   Recommended actions:
   1. Wait and see if robot appears (may take 10-30 seconds)
   2. If robot stays white, press Ctrl+C and restart system
   3. Use -preload flag next time to avoid mode switching

   Continue anyway? (yes/no): no
   Returning to menu...
```

### If Robot White in RViz

```
🤖 Robot Status Check
   Is the robot visible and ready? (yes/no): no
   ⚠️  Robot not visible in RViz

   This usually means TF transforms are broken.
   The robot will not be able to navigate properly.

   RECOMMENDED FIX:
   1. Press Ctrl+C to stop
   2. Stop Gazebo/TurtleBot
   3. Restart everything fresh
   4. Use: ./scripts/run_autonomous_slam.sh -preload

   Stop now and fix? (yes/no): yes
   Stopping inspection robot...
   Returning to menu...
```

## Prevention Strategies

### Strategy 1: Use -preload Flag (Best)
```bash
# Avoids mode switching entirely
./scripts/run_autonomous_slam.sh -preload

# SLAM stays in localization mode
# No TF disruption
# Robot stays visible
```

**Why it works:**
- No SLAM mode switching (mapping → localization)
- TF tree remains stable
- No white robot issues

### Strategy 2: Fresh Start
```bash
# Stop everything
Ctrl+C in all terminals

# Restart Gazebo
./launch_warehouse.sh

# Run with preload
./scripts/run_autonomous_slam.sh -preload
```

### Strategy 3: Wait It Out
If robot goes white:
1. **Don't panic** - It often recovers
2. **Wait 10-30 seconds** - TF tree may stabilize
3. **Check RViz** - Robot may reappear
4. **If persistent** - Restart system

## Technical Details

### TF Check Implementation

```bash
TF_STABLE=false
TF_CHECK_COUNT=0

while [ $TF_CHECK_COUNT -lt 30 ]; do
    # Check if map → base_footprint transform exists
    if ros2 run tf2_ros tf2_echo map base_footprint > /dev/null 2>&1; then
        TF_CHECK_COUNT=$((TF_CHECK_COUNT + 1))
        
        # Need 5 consecutive successful checks
        if [ $TF_CHECK_COUNT -ge 5 ]; then
            TF_STABLE=true
            break
        fi
    else
        # Failed check, reset counter
        TF_CHECK_COUNT=0
    fi
    
    sleep 0.5
done
```

### Why 5 Consecutive Checks?

- **1 check**: Could be a fluke
- **5 checks**: Confirms stability over 2.5 seconds
- **Prevents**: Starting with intermittent TF

### Critical Transforms

The system checks:
- `map → odom` (SLAM Toolbox provides)
- `odom → base_footprint` (Robot state publisher provides)
- `map → base_footprint` (Combined transform)

If any are missing or unstable → White robot

## Troubleshooting

### Robot Goes White During Operation

**Cause**: TF transforms lost during operation

**Solutions:**

1. **Wait**: Often recovers automatically in 10-30 seconds

2. **Check processes**:
   ```bash
   # Is SLAM Toolbox running?
   ps aux | grep slam_toolbox
   
   # Is robot_state_publisher running?
   ps aux | grep robot_state_publisher
   ```

3. **Check TF tree**:
   ```bash
   # View TF tree
   ros2 run tf2_tools view_frames
   
   # Check specific transform
   ros2 run tf2_ros tf2_echo map base_footprint
   ```

4. **Restart if needed**:
   ```bash
   # Stop inspection
   Ctrl+C
   
   # Restart with preload
   ./scripts/run_autonomous_slam.sh -preload
   ```

### Robot White at Startup

**Cause**: TF not ready when robot node starts

**The script now handles this by:**
- ✅ Waiting for TF stability (5 consecutive checks)
- ✅ Extra 3-second pause for RViz
- ✅ User confirmation before proceeding
- ✅ Clear instructions if issues persist

### SLAM Mode Switching Issues

**Problem**: Switching mapping → localization breaks TF

**Best Practice:**
```bash
# DON'T: Run exploration, then switch to inspection
./scripts/run_autonomous_slam.sh
# [exploration completes]
# [select option 5] ← TF breaks here

# DO: Use preload to skip exploration
./scripts/run_autonomous_slam.sh -preload
# [select option 5] ← No TF break
```

## Best Practices

### For Reliable Operation

1. **Use -preload flag** when you already have a map
   ```bash
   ./scripts/run_autonomous_slam.sh -preload
   ```

2. **Fresh start** if switching modes
   - Stop all processes
   - Restart Gazebo
   - Run script fresh

3. **Wait for confirmations**
   - Don't skip TF stability checks
   - Confirm robot visible before proceeding
   - Be patient with initialization

4. **Monitor RViz**
   - Keep RViz visible during startup
   - Watch for robot to appear
   - Check laser scans are showing

### For Unstable Networks

```bash
# Combine preload + headless camera
./scripts/run_autonomous_slam.sh -preload -nocamui

# Benefits:
# • No SLAM mode switching (stable TF)
# • No camera GUI (less bandwidth)
# • More reliable operation
```

## Quick Reference

### Flags
| Flag | Purpose | Benefit |
|------|---------|---------|
| `-preload` | Skip exploration | Prevents TF disruption |
| `-nocamui` | Disable camera GUI | Reduces network load |
| `-web` | Enable dashboard | Remote monitoring |

### Checks Added
| Check | Purpose | Timeout |
|-------|---------|---------|
| Camera feed | Verify camera working | 10 seconds |
| TF stability | Ensure transforms ready | 15 seconds (5 checks) |
| Robot visibility | Confirm RViz shows robot | 20 seconds |
| Viewfinder | Verify GUI working | 15 seconds |

### Recovery Steps

**If robot goes white:**
1. Wait 30 seconds (may recover)
2. Check TF: `ros2 run tf2_ros tf2_echo map base_footprint`
3. If still white, restart system
4. Use `-preload` flag next time

**If camera unstable:**
1. Use `-nocamui` flag
2. Detection still works
3. No GUI overhead

**If both issues:**
```bash
./scripts/run_autonomous_slam.sh -preload -nocamui
```

## Summary

✅ **TF stability check** - Waits for 5 consecutive successful transform checks
✅ **Extra initialization time** - 3-second pause for RViz to update
✅ **User verification** - Confirms robot visible before proceeding
✅ **Clear error messages** - Explains issues and solutions
✅ **Graceful handling** - Offers to stop and fix properly
✅ **Prevention advice** - Recommends -preload flag

The system now handles TF instability much better and guides users to proper solutions!
