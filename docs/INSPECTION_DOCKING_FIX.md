# Inspection Mode Docking Fix

## Problem
Inspection exploration mode was getting stuck in a loop when trying to return home:

```
[INFO] Planning path home from (0.44, 0.49)...
[INFO] Path set with 3 waypoints, goal at (0.36, 0.45)
[INFO] Reached goal! Distance: 0.088m
[INFO] Planning path home from (0.44, 0.49)...
[ERROR] Cannot plan path home!
```

The robot was close to home (~0.65m) but not close enough to trigger the "already at home" check (0.15m threshold). Path planning would fail or produce tiny paths, causing an infinite loop.

## Root Cause
Inspection mode was missing the **line-of-sight docking logic** that delivery and exploration modes already had. When the robot gets close to home (within 75cm) and has a clear line of sight, it should switch to direct docking mode instead of relying on path planning.

## Solution
Added the same line-of-sight docking logic from DeliveryRobot and AutonomousExplorationRobot:

### Before Path Planning
```cpp
// Check if we're close enough to switch to docking mode (within 75cm)
if (dist_to_home < DOCKING_DISTANCE * 1.5) {  // Within 75cm
    bool has_clear_path = hasLineOfSight(current_pose.position, home_position, *current_map);
    
    if (has_clear_path) {
        RCLCPP_INFO(node_->get_logger(), 
                   "Close to home (%.2fm) with clear line of sight - switching to docking mode", 
                   dist_to_home);
        in_docking_mode_ = true;
        motion_controller_->clearPath();
        preciseDocking(current_pose, dist_to_home);
        return;
    }
}
```

### After Path Planning Fails
```cpp
if (path.empty()) {
    // If path planning fails but we're close with line of sight, try docking
    if (dist_to_home < DOCKING_DISTANCE * 1.5) {  // Within 75cm
        bool has_clear_path = hasLineOfSight(current_pose.position, home_position, *current_map);
        
        if (has_clear_path) {
            RCLCPP_INFO(node_->get_logger(), 
                       "Path planning failed but %.2fm from home with clear line of sight - switching to docking mode", 
                       dist_to_home);
            in_docking_mode_ = true;
            motion_controller_->clearPath();
            preciseDocking(current_pose, dist_to_home);
            return;
        }
    }
    // ... existing fallback logic
}
```

## How It Works

1. **Distance Check**: When robot is within 75cm of home (DOCKING_DISTANCE * 1.5)
2. **Line of Sight Check**: Uses `hasLineOfSight()` to verify no obstacles between robot and home
3. **Switch to Docking**: If clear path exists, switches to `preciseDocking()` mode
4. **Direct Navigation**: Robot drives straight to home without path planning

This matches the behavior already working in delivery and exploration modes.

## Benefits

- ✅ No more infinite loops when close to home
- ✅ Smoother final approach to home position
- ✅ Consistent behavior across all robot modes
- ✅ Handles cases where path planner struggles with small distances

## Files Modified
- `turtlebot3_ws/src/turtlebot3_simulations/warehouse_robot_system/src/Robot/InspectionRobot.cpp`

## Testing
Run inspection exploration mode and verify it successfully returns home without getting stuck:

```bash
./scripts/run_autonomous_slam.sh -nocamui
# Select option 5 (Inspection Exploration)
# Wait for patrol to complete
# Verify robot returns home smoothly
```

## Status
✅ Fixed and built
✅ Ready for testing
