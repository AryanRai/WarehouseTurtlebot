# Inspection System - Complete Guide

## Overview

The inspection system has **two modes** for damage detection using AprilTags:

1. **Inspection Exploration Mode** (Option 5) - Autonomous discovery
2. **Inspection Mode** (Option 6) - Visit pre-defined sites

## Mode Comparison

| Feature | Inspection Exploration | Inspection Mode |
|---------|----------------------|-----------------|
| **Purpose** | Discover new damage sites | Visit known damage sites |
| **Navigation** | Autonomous frontier exploration | Targeted navigation to sites |
| **AprilTag Detection** | Continuous during exploration | At specific locations only |
| **Output** | damage_sites.yaml (discovered) | inspection_log.csv (results) |
| **When to Use** | First time / unknown damages | After sites are known |
| **Map Required** | No (creates map) | Yes (uses existing map) |

## Option 5: Inspection Exploration Mode 🔍

### What It Does
- Robot explores the warehouse autonomously
- Camera continuously monitors for AprilTags
- When tag detected, location is automatically saved
- Creates `damage_sites.yaml` with discovered sites

### Workflow
```
1. Start exploration with camera active
2. Robot navigates using frontier exploration
3. AprilTag detector runs in background
4. Tag detected → Position saved to damage_sites.yaml
5. Exploration continues until complete
6. Return home with list of discovered sites
```

### Current Implementation Status
**Partially Implemented** - The infrastructure is ready:
- ✅ AprilTag detector works
- ✅ Exploration system works
- ✅ Camera integration works
- 🚧 Automatic site marking needs integration

### How to Use (Current Approach)
```bash
# 1. Start system
./scripts/run_autonomous_slam.sh

# 2. Select option [5] Inspection Exploration Mode

# 3. Choose [1] to run exploration with AprilTag detection

# 4. Monitor detections in another terminal:
ros2 topic echo /apriltag_detections

# 5. After exploration, manually mark sites near detected tags
```

### Future Enhancement
Full automation would add:
- Automatic position logging when tag detected
- Pause exploration to verify tag
- Resume exploration after logging
- Generate damage_sites.yaml automatically

## Option 6: Inspection Mode 📋

### What It Does
- Navigates to pre-defined damage sites
- Approaches each site precisely
- Reads AprilTag ID at site
- Logs results to `inspection_log.csv`
- Returns home after all inspections

### Workflow
```
1. Load damage_sites.yaml (pre-defined sites)
2. Plan optimal route (TSP or ordered)
3. Navigate to first site
4. Approach within 30cm
5. Read AprilTag ID (5 second timeout)
6. Log result (success/failure)
7. Move to next site
8. Return home when complete
```

### Current Implementation Status
**Fully Implemented** ✅
- ✅ Site navigation
- ✅ Precise docking
- ✅ AprilTag reading
- ✅ CSV logging
- ✅ TSP optimization
- ✅ Error handling

### How to Use
```bash
# 1. Define damage sites (manual or from exploration)
./scripts/run_autonomous_slam.sh
# Select [6] Inspection Mode

# 2. In RViz, click points near AprilTags
# Use "Publish Point" tool

# 3. Save sites
./scripts/inspection_commands.sh save

# 4. Start inspections
./scripts/inspection_commands.sh start

# 5. Monitor progress
./scripts/inspection_commands.sh status
```

## Complete Workflow Example

### Scenario: New Warehouse Inspection

**Step 1: Initial Exploration** (Create Map)
```bash
./scripts/run_autonomous_slam.sh
# Select [1] Exploration Mode
# Wait for completion
```

**Step 2: Inspection Exploration** (Discover Damages)
```bash
# Select [5] Inspection Exploration Mode
# Choose [1] Run exploration with AprilTag detection
# Monitor: ros2 topic echo /apriltag_detections
```

**Step 3: Mark Damage Sites** (Manual for now)
```bash
# Note detected tag locations from logs
# Select [6] Inspection Mode
# Click points in RViz near detected tags
# Save: ./scripts/inspection_commands.sh save
```

**Step 4: Run Inspections** (Verify and Log)
```bash
# Start: ./scripts/inspection_commands.sh start
# Monitor: ./scripts/inspection_commands.sh status
# View results: ./scripts/inspection_commands.sh log
```

**Step 5: Save Everything**
```bash
# Select [7] Save Current Map
# Enter name: "warehouse_with_damages"
# Everything saved together
```

## Menu Structure

```
🤖 SELECT ROBOT MODE
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[1] 🗺️  EXPLORATION MODE
    Create new map from scratch

[2] 📍 DEFINE DELIVERY ZONES
    Mark delivery locations

[3] 📦 DELIVERY MODE
    Multi-point delivery operations

[4] 💾 SAVE CURRENT MAP
    Save map with custom name

[5] 🔍 INSPECTION EXPLORATION MODE  ← NEW!
    Discover damage sites automatically

[6] 📋 INSPECTION MODE  ← RENAMED (was [5])
    Visit pre-defined damage sites

[7] 💾 SAVE CURRENT MAP  ← MOVED (was [4])
    Save map with custom name

[8] ❌ EXIT  ← MOVED (was [6])
    Shutdown system
```

## Files Generated

### Inspection Exploration Mode
- `damage_sites.yaml` - Discovered damage locations
- `warehouse_map_*.yaml` - Map with damage sites marked

### Inspection Mode
- `inspection_log.csv` - Inspection results
- Format: `Timestamp,SiteName,AprilTagID,Position_X,Position_Y,Distance(m),Time(s),Status,Notes`

## Assignment Requirements Met

### Inspection Robot Requirements ✅
- ✅ **Has camera + LiDAR + odometry**
- ✅ **Locates damages in warehouse** (via AprilTags)
- ✅ **Saves damage records to disk** (inspection_log.csv)

### Extensions Implemented ✅
- ✅ **Autonomous mapping** (exploration mode)
- ✅ **Navigate to damage sites** (inspection mode)
- ✅ **Intelligent scheduling** (TSP optimization)

## Testing Checklist

### Test Inspection Exploration
- [ ] Launch Gazebo with AprilTags
- [ ] Start inspection exploration mode
- [ ] Verify AprilTag detector running
- [ ] Check detections: `ros2 topic echo /apriltag_detections`
- [ ] Verify exploration completes
- [ ] Check damage_sites.yaml created

### Test Inspection Mode
- [ ] Load map with AprilTags
- [ ] Define damage sites in RViz
- [ ] Save sites
- [ ] Start inspections
- [ ] Verify robot visits all sites
- [ ] Check AprilTag IDs logged correctly
- [ ] Verify inspection_log.csv created

## Troubleshooting

### Inspection Exploration Issues

**Problem**: No AprilTags detected during exploration
- Check camera: `ros2 topic hz /camera/image_raw`
- Check detector: `ros2 node list | grep apriltag`
- Verify tags in Gazebo are visible and upright

**Problem**: Sites not saved automatically
- Current limitation - manual marking required
- Use inspection mode (option 6) to mark sites

### Inspection Mode Issues

**Problem**: Robot doesn't approach sites
- Check damage_sites.yaml exists
- Verify sites are on accessible areas
- Check map is loaded correctly

**Problem**: AprilTag not detected at site
- Verify tag is within 2m
- Check tag is facing robot
- Increase timeout if needed

## Related Documentation

- [Inspection Robot Implementation](INSPECTION_ROBOT_IMPLEMENTATION.md)
- [AprilTag Setup](APRILTAG_GAZEBO_SETUP.md)
- [AprilTag Quick Setup](APRILTAG_QUICK_SETUP.md)
- [Assignment Requirements](../Docs/assignment.md)

## Quick Commands

```bash
# Inspection exploration
./scripts/run_autonomous_slam.sh  # Select [5]

# Inspection mode
./scripts/inspection_commands.sh save    # Save sites
./scripts/inspection_commands.sh start   # Start inspections
./scripts/inspection_commands.sh status  # Monitor
./scripts/inspection_commands.sh log     # View results

# Test AprilTag setup
./scripts/test_apriltag_setup.sh
```
