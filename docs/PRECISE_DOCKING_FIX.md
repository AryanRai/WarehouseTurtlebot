# Precise Docking Mode Fix

## Problem
When returning home, the robot would get very close (within ~10cm) but not quite reach the exact position. This caused it to keep trying to follow the path, overshoot, and enter recovery loops repeatedly.

## Root Cause
The robot was using the same path-following controller all the way to home, which:
- Has momentum and overshoots the target
- Uses a relatively loose tolerance (10cm)
- Doesn't provide precise positioning control

## Solution: Two-Phase Approach

### Phase 1: Path Following (> 50cm from home)
- Uses the normal A* path planner and motion controller
- Efficient for long-distance navigation
- Handles obstacles and complex paths

### Phase 2: Precise Docking (< 50cm from home)
When the robot gets within 50cm of home, it switches to a precise docking mode:

1. **Alignment Phase**: If heading is off by more than 10°
   - Stops forward motion
   - Rotates in place to face home
   - Uses moderate angular speed (0.3 rad/s)

2. **Approach Phase**: Once aligned
   - Moves forward slowly (0.05 m/s)
   - Applies gentle angular correction
   - Continues until within 5cm of home

### Success Criteria
- **Docking Distance**: 50cm (enters docking mode)
- **Home Tolerance**: 5cm (declares success)
- **Alignment Tolerance**: 10° (switches from rotation to approach)

## Key Changes

### AutonomousExplorationRobot.hpp
- Added `in_docking_mode_` flag
- Added `preciseDocking()` method
- Added docking constants:
  - `DOCKING_DISTANCE = 0.5m`
  - `HOME_TOLERANCE = 0.05m`
  - `DOCKING_LINEAR_SPEED = 0.05 m/s`
  - `DOCKING_ANGULAR_SPEED = 0.3 rad/s`

### AutonomousExplorationRobot.cpp
- Implemented `preciseDocking()` method with:
  - Angle calculation to home
  - Two-phase control (align then approach)
  - Direct velocity commands
- Modified `returnToHome()` to:
  - Check distance and enter docking mode at 50cm
  - Use tighter tolerance (5cm) for success
  - Clear path planner when entering docking mode

## Benefits
1. **Precise positioning**: Gets within 5cm of home instead of 10cm
2. **No overshooting**: Slow, controlled approach prevents momentum issues
3. **No recovery loops**: Reaches target before triggering recovery behavior
4. **Smooth transition**: Seamlessly switches from path following to docking

## Testing
After building, test by:
1. Starting autonomous exploration
2. Letting it explore until it decides to return home
3. Observing the transition to docking mode at ~50cm
4. Verifying it stops within 5cm of (0, 0)

Expected log output:
```
[INFO] Entering precise docking mode (0.450m from home)
[INFO] Docking: Aligning (angle error: 15.3°)
[INFO] Docking: Moving forward (0.234m remaining)
[INFO] Successfully returned to home position! (distance: 0.042m)
```
