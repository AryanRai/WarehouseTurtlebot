# Line-of-Sight Robot Footprint Fix

## Problem

The robot was colliding with walls during docking even though the line-of-sight check reported a clear path. This happened when:

1. Robot traveling diagonally toward a zone
2. Direct line from robot center to zone is clear
3. BUT wall is beside the path
4. Robot's body collides with wall during diagonal movement

### Example Scenario
```
    Wall
    ████
    ████  ← Wall beside path
    ████
         \
          \  ← Robot traveling diagonally
           \
            ● Zone (clear line of sight to center)
```

The center line is clear, but the robot's body hits the wall!

## Root Cause

The `hasLineOfSight()` function only checked if cells along the direct line were walkable:

```cpp
// OLD CODE - Only checked center line
while (true) {
    GridCell current = {x0, y0};
    if (!isCellWalkable(current)) {
        return false;  // Only checks single cell
    }
    // Move to next cell on line...
}
```

This didn't account for the robot's physical width (~17.8cm diameter).

## Solution

Enhanced `hasLineOfSight()` to check cells around the line, accounting for robot footprint:

```cpp
// NEW CODE - Checks robot footprint
const int robot_radius_cells = 2;  // ~0.09m radius = 2 cells at 0.05m resolution

while (true) {
    // Check current cell AND surrounding cells (robot footprint)
    for (int dx = -robot_radius_cells; dx <= robot_radius_cells; ++dx) {
        for (int dy = -robot_radius_cells; dy <= robot_radius_cells; ++dy) {
            // Only check cells within circular footprint
            if (dx*dx + dy*dy <= robot_radius_cells * robot_radius_cells) {
                GridCell check_cell = {x0 + dx, y0 + dy};
                if (!isCellWalkable(check_cell)) {
                    return false;  // Would hit robot body
                }
            }
        }
    }
    // Move to next cell on line...
}
```

### Robot Footprint Calculation

- **TurtleBot3 Burger diameter**: 0.178m
- **Robot radius**: 0.089m
- **Map resolution**: 0.05m per cell
- **Radius in cells**: 0.089m / 0.05m ≈ 1.78 cells
- **Safety margin**: Round up to 2 cells

### Circular Footprint Check

Instead of checking a square around each point, we check a circle:

```
  □ □ ■ □ □
  □ ■ ■ ■ □
  ■ ■ ● ■ ■  ← Robot center (●)
  □ ■ ■ ■ □
  □ □ ■ □ □

■ = Checked cells (within radius)
□ = Not checked (outside radius)
● = Robot center on path
```

This more accurately represents the robot's circular shape.

## Benefits

✅ **Prevents Wall Collisions**: Detects walls beside diagonal paths
✅ **Accurate Footprint**: Considers actual robot dimensions
✅ **Circular Check**: Matches robot's round shape
✅ **Safety Margin**: 2-cell radius provides buffer
✅ **Diagonal Safety**: Catches obstacles that center-line check misses

## Behavior After Fix

### Before (Center-Line Only)
```
Checking line of sight...
  Center line clear? YES
  → Entering docking mode
  → Robot moves diagonally
  → COLLISION with wall beside path!
```

### After (With Footprint)
```
Checking line of sight...
  Center line clear? YES
  Robot footprint clear? NO (wall detected beside path)
  → Line of sight: BLOCKED
  → Staying with path planner (safer route)
  → No collision!
```

## Edge Cases Handled

1. **Diagonal Movement Near Walls**: Now detected
2. **Narrow Passages**: Robot won't try to dock through tight spaces
3. **Corner Obstacles**: Footprint check catches corner collisions
4. **Doorways**: Won't dock through doorways that are too narrow

## Performance Impact

- **Additional checks per cell**: ~13 cells (circular footprint)
- **Typical path length**: 10-50 cells
- **Total extra checks**: 130-650 cells
- **Performance**: Negligible (< 1ms for typical paths)

## Testing

To verify the fix:

1. Place a zone near a wall
2. Position robot so diagonal path to zone passes close to wall
3. Observe: Robot should NOT enter docking mode
4. Robot uses path planner instead (safer route around wall)

Expected log:
```
[INFO] Close to zone (0.45m) but obstacle detected - continuing with path planner
```

Instead of:
```
[INFO] Entering precise zone docking mode
[ERROR] Collision detected!
```

## Related Parameters

```cpp
// Robot dimensions
const double ROBOT_DIAMETER = 0.178;  // meters
const double ROBOT_RADIUS = 0.089;    // meters

// Map resolution
const double MAP_RESOLUTION = 0.05;   // meters per cell

// Safety margin
const int ROBOT_RADIUS_CELLS = 2;     // cells (with buffer)
```

## Future Enhancements

- Adjustable safety margin based on speed
- Elliptical footprint for rectangular robots
- Dynamic footprint based on robot orientation
- Inflation layer integration with costmap

---

This fix ensures the robot only attempts docking when there's truly enough clearance for its entire body, not just its center point!
