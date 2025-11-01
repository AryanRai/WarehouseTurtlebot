# TF2 Transform Issues When Switching Modes - Fix Guide

## Problem Description

When switching between different robot modes (especially from Delivery/Localization to Exploration/Mapping), the robot may appear **white/frozen** in Gazebo and not respond to commands. This is caused by stale TF2 transforms that persist when SLAM Toolbox is restarted.

### Symptoms
- Robot model turns white in Gazebo
- Robot doesn't move despite commands being sent
- Log shows: `Waiting for valid map and pose from SLAM...`
- TF tree has broken or stale transforms
- `/cmd_vel` topic receives no messages

### Root Cause
When SLAM Toolbox switches from localization mode to mapping mode (or vice versa), it publishes different TF transforms. If the old transforms aren't properly cleared, the TF tree becomes inconsistent, causing:
- Pose estimation failures
- Map-to-odom transform conflicts
- Motion controller unable to compute velocities

## Solutions

### Solution 1: Full System Restart (RECOMMENDED)

This is the most reliable method to ensure clean TF transforms:

```bash
# 1. Stop the autonomous SLAM script (Ctrl+C)

# 2. Stop Gazebo (Ctrl+C in Gazebo terminal)

# 3. Wait a few seconds for everything to shut down
sleep 3

# 4. Restart Gazebo
./launch_warehouse.sh

# 5. Wait for Gazebo to fully load (robot should appear normal)

# 6. Start autonomous SLAM
./scripts/run_autonomous_slam.sh

# 7. Select your desired mode (e.g., Exploration Mode)
```

### Solution 2: Quick Restart Script

Use the provided restart script:

```bash
./scripts/restart_for_exploration.sh
```

This script will:
1. Stop all ROS nodes
2. Stop Gazebo
3. Provide instructions for clean restart

### Solution 3: Manual TF Reset (Advanced)

If you want to try switching modes without full restart:

```bash
# 1. Stop SLAM Toolbox
pkill -f slam_toolbox

# 2. Wait for TF to clear (important!)
sleep 5

# 3. Clear any TF cache
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
sleep 1
pkill -f static_transform_publisher

# 4. Restart SLAM Toolbox in desired mode
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

**Note:** This method is less reliable and may still have issues.

## Prevention

### Best Practices

1. **Plan your workflow**: Decide which mode you need before starting
2. **Use preload mode**: If you have an existing map, use `-preload` flag:
   ```bash
   ./scripts/run_autonomous_slam.sh -preload
   ```
3. **Avoid mode switching**: Complete one task before switching modes
4. **Save progress**: Always save your map before switching modes

### Starting Fresh Each Time

For most reliable operation:

```bash
# Always start with a clean slate
./launch_warehouse.sh

# Then choose your mode
./scripts/run_autonomous_slam.sh

# Select mode 1, 2, or 3 based on your needs
```

## Mode-Specific Guidance

### Exploration Mode (Creating New Map)
- **Always** start with fresh Gazebo instance
- Don't switch to this mode from other modes
- Let exploration complete fully before switching

### Delivery Mode (Using Existing Map)
- Use `-preload` flag to load existing map
- Don't switch from exploration mode
- Restart system if coming from exploration

### Zone Definition Mode
- Can be used after exploration completes
- Or use with `-preload` flag
- Relatively safe to switch to from delivery mode

## Script Improvements

The `run_autonomous_slam.sh` script now includes:

1. **Warning message** when attempting to switch to exploration mode
2. **Confirmation prompt** before mode switch
3. **Extended wait time** (5 seconds) for TF to clear
4. **Recommendation** to restart system instead

### Updated Mode Switch Flow

```
User selects Exploration Mode
    ↓
Script shows TF warning
    ↓
User confirms or cancels
    ↓
If confirmed:
  - Stop SLAM Toolbox
  - Wait 3 seconds for TF clear
  - Start SLAM in mapping mode
  - Wait 5 seconds for initialization
    ↓
If cancelled:
  - Return to menu
  - User can restart system
```

## Troubleshooting

### Robot Still White After Restart?

1. **Check Gazebo is fully loaded**:
   ```bash
   gz topic -l | grep /model/turtlebot3
   ```
   Should show robot topics

2. **Check TF tree**:
   ```bash
   ros2 run tf2_tools view_frames
   ```
   Look for broken connections

3. **Check SLAM Toolbox**:
   ```bash
   ros2 topic echo /map --once
   ```
   Should receive map data

4. **Check robot bridge**:
   ```bash
   ros2 topic list | grep cmd_vel
   ```
   Should show `/cmd_vel` topic

### Commands Not Reaching Robot?

```bash
# Test if robot receives commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" --once

# Check if Gazebo bridge is running
ps aux | grep ros_gz_bridge
```

### SLAM Not Publishing Pose?

```bash
# Check SLAM Toolbox status
ros2 node list | grep slam

# Check pose topic
ros2 topic echo /pose --once

# Restart SLAM Toolbox
pkill -f slam_toolbox
sleep 3
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
```

## Technical Details

### TF Tree Structure

**Normal (Healthy) TF Tree:**
```
map → odom → base_footprint → base_link → sensors
```

**Broken TF Tree (After Bad Mode Switch):**
```
map → odom (stale)
map → odom (new)  ← Conflict!
base_footprint (orphaned)
```

### Why Full Restart Works

1. **Gazebo reset**: Clears all simulation state
2. **ROS node restart**: Fresh node graph
3. **TF buffer clear**: No stale transforms
4. **SLAM fresh start**: Clean map-odom transform

## Quick Reference

| Situation | Solution | Time |
|-----------|----------|------|
| Starting fresh | `./launch_warehouse.sh` then `./scripts/run_autonomous_slam.sh` | 30s |
| Robot white/frozen | Full restart (stop Gazebo, restart) | 1min |
| Switching modes | Use restart script or full restart | 1min |
| After exploration | Can switch to delivery/zones without restart | 0s |
| After delivery | Restart for exploration | 1min |

## Related Files

- `scripts/run_autonomous_slam.sh` - Main script with TF warnings
- `scripts/restart_for_exploration.sh` - Quick restart helper
- `docs/QUICK_START.md` - General usage guide
- `docs/TF_BREAK_FIX.md` - Original TF issue documentation

## Summary

**The golden rule**: When in doubt, restart Gazebo and the autonomous SLAM script. This ensures clean TF transforms and prevents the white robot issue. The extra 30-60 seconds for a restart is worth avoiding TF debugging!
