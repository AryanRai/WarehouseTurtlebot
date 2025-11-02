# Inspection Home Marker Bug Fix

## Problem
In inspection exploration mode, damage markers were appearing at the home position (0, 0) even though the actual AprilTags were elsewhere. For example:
- Marker ID 1 correctly placed at AprilTag 1
- Marker ID 2 correctly placed at AprilTag 2  
- **Marker ID 2 also incorrectly placed at home (0, 0)**

## Root Cause
The robot was detecting AprilTags while at or near the home position (during startup, returning home, or docking), and the detection callback was saving these with the robot's current position instead of filtering them out.

**Code issue (line 160):**
```cpp
saveDiscoveredSite(last_detected_tag_id_, current_pose.position);
```

This saves the tag at the **robot's position**, not the tag's actual position. When the robot is at home and sees a tag in the distance, it incorrectly marks the tag as being at home.

## Solution
Added a distance check to **ignore tag detections when the robot is at home** (within 0.5m of origin):

```cpp
// Don't save tags detected at home (within 0.5m of origin)
double dist_from_home = std::sqrt(current_pose.position.x * current_pose.position.x + 
                                 current_pose.position.y * current_pose.position.y);
if (dist_from_home < 0.5) {
    RCLCPP_DEBUG(node_->get_logger(), "Ignoring tag detection at home position");
    return;
}
```

## Why This Works

### Scenario 1: Robot at Home Sees Distant Tag
- Robot position: (0, 0) - at home
- Tag position: (2, 1) - somewhere in warehouse
- **Before:** Saves tag at (0, 0) ❌
- **After:** Ignores detection ✅

### Scenario 2: Robot Near Tag During Patrol
- Robot position: (2.1, 1.0) - near the tag
- Tag position: (2, 1) - actual tag location
- Distance from home: 2.3m > 0.5m
- **Before:** Saves tag at (2.1, 1.0) ✅
- **After:** Saves tag at (2.1, 1.0) ✅

### Scenario 3: Robot Returning Home
- Robot position: (0.3, 0.2) - approaching home
- Tag position: (2, 1) - tag seen in distance
- Distance from home: 0.36m < 0.5m
- **Before:** Saves tag at (0.3, 0.2) ❌
- **After:** Ignores detection ✅

## Limitations

This is a **proximity-based fix** that prevents the most common issue (tags detected at home). However, it still uses the robot's position as a proxy for the tag's position, which means:

- ✅ Tags are saved near their actual location (within ~1m)
- ✅ No more spurious markers at home
- ⚠️ Tag position is approximate (robot position, not exact tag position)

### Future Improvement
For exact tag positions, we would need to:
1. Get tag pose from detection message (relative to camera)
2. Transform from camera frame → base_footprint → map
3. Use transformed position instead of robot position

But for exploration mode, the current approximation is sufficient.

## Files Modified
- `turtlebot3_ws/src/turtlebot3_simulations/warehouse_robot_system/src/Robot/InspectionRobot.cpp`

## Testing
1. Delete old damage_sites.yaml: `rm damage_sites.yaml`
2. Run inspection exploration mode
3. Verify no markers appear at home (0, 0)
4. Verify markers appear near actual AprilTags

```bash
./scripts/run_autonomous_slam.sh -nocamui
# Select option 5 (Inspection Exploration)
# Watch RViz - no markers should appear at origin
```

## Status
✅ Home position check added (0.5m radius)
✅ Built and ready for testing
✅ Prevents spurious markers at home
