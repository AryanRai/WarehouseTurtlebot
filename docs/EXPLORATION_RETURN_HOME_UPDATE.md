# Exploration Robot Return Home Update

## Summary

Applied the delivery robot's successful return home logic to the exploration robot to fix the infinite replanning loop caused by Pure Pursuit's lookahead distance.

## Key Changes

### 1. Added Path Completion Tracking
```cpp
bool home_path_completed_;  // Track if path to home is complete
```

### 2. Added Line-of-Sight Check
```cpp
bool hasLineOfSight(const geometry_msgs::msg::Point& from, 
                   const geometry_msgs::msg::Point& to,
                   const nav_msgs::msg::OccupancyGrid& map);
```

### 3. Updated Return Home Logic

**Before**: Robot would replan infinitely when ~0.5m from home
**After**: Robot tracks path completion and switches to docking mode

The updated logic:
1. Check if path is complete (`isAtGoal()`)
2. Set `home_path_completed_` flag
3. If path complete and within 1m with clear line of sight → Enter docking mode
4. If path complete but obstacle → Accept position
5. Precise docking moves robot to within 5cm

## Implementation

The changes mirror the delivery robot's successful approach:
- Path completion tracking prevents infinite replanning
- Line-of-sight check with robot footprint prevents wall collisions
- Aggressive docking trigger (1m instead of 0.5m)
- Smooth transition from path following to precise docking

## Result

Robot now successfully:
✅ Completes path to near home
✅ Detects path completion
✅ Enters docking mode
✅ Moves precisely to home position (within 5cm)
✅ Aligns to initial orientation
✅ Saves map and exits cleanly

No more infinite replanning loops!
