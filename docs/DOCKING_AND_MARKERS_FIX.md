# Docking Line-of-Sight and Marker Visibility Fixes

## Issue 1: Docking Mode Obstacle Collision

### Problem
Robot entered docking mode when within 50cm of home, even if there was a wall between the robot and home position. This caused the robot to drive straight into obstacles.

### Solution
Added line-of-sight check using Bresenham's line algorithm before entering docking mode.

```cpp
bool DeliveryRobot::hasLineOfSight(const geometry_msgs::msg::Point& from, 
                                   const geometry_msgs::msg::Point& to,
                                   const nav_msgs::msg::OccupancyGrid& map) {
    // Use Bresenham's line algorithm to check if path is clear
    // Returns true only if all cells between from and to are walkable
}
```

### New Docking Logic

**Before:**
```cpp
if (distance_to_home < DOCKING_DISTANCE) {
    // Enter docking mode immediately
    preciseDocking(current_pose, distance_to_home);
}
```

**After:**
```cpp
if (distance_to_home < DOCKING_DISTANCE) {
    bool has_clear_path = hasLineOfSight(current_pose.position, home_position, *current_map);
    
    if (has_clear_path) {
        // Safe to enter docking mode
        preciseDocking(current_pose, distance_to_home);
    } else {
        // Obstacle detected - keep using path planner
        RCLCPP_INFO("Close to home but obstacle detected - continuing with path planner");
        // Fall through to normal path planning
    }
}
```

### Benefits

1. **Safety**: Robot won't drive into walls
2. **Smart Navigation**: Uses path planner to navigate around obstacles
3. **Seamless Transition**: Enters docking mode only when safe
4. **No Manual Intervention**: Automatically handles complex scenarios

### Example Scenario

```
Robot at (2.0, 0.0), Home at (0.0, 0.0), Wall at (1.0, 0.0)

Distance: 2.0m (> 0.5m) → Use path planner
Distance: 0.4m but wall detected → Keep using path planner
Robot navigates around wall
Distance: 0.4m with clear path → Enter docking mode
```

## Issue 2: Zone Markers Not Visible in RViz

### Problem
Zone markers were being published to `/delivery_zones/markers` but not appearing in RViz because the MarkerArray display wasn't configured.

### Solutions

#### 1. Immediate Marker Publishing
Added immediate publishing of existing zones on startup:

```cpp
// Publish existing zones immediately
if (!zones_.empty()) {
    publishMarkers();
    RCLCPP_INFO("Published %zu existing zone markers", zones_.size());
}
```

#### 2. Clear Instructions
Updated script to include RViz setup instructions:

```
2️⃣  Add MarkerArray display if not visible:
    • Click 'Add' button in RViz
    • Select 'MarkerArray'
    • Set Topic to: /delivery_zones/markers
```

#### 3. Better Logging
Added logging to show marker topic:

```cpp
RCLCPP_INFO("Publishing markers to: /delivery_zones/markers");
```

### RViz Setup

To see zone markers in RViz:

1. **Add MarkerArray Display:**
   - Click "Add" button (bottom left)
   - Select "By topic" tab
   - Find `/delivery_zones/markers`
   - Click "OK"

2. **Verify Display:**
   - Check "MarkerArray" is enabled in Displays panel
   - Topic should show: `/delivery_zones/markers`
   - Status should be "OK"

3. **Markers Appearance:**
   - Colored cylinders at zone positions
   - Text labels showing zone names
   - Colors cycle: Red, Green, Blue

### Marker Properties

```cpp
// Cylinder
- Diameter: 0.3m
- Height: 0.3m
- Position Z: 0.15m (half height above ground)
- Alpha: 0.7 (semi-transparent)

// Text
- Height: 0.15m
- Position Z: 0.5m (above cylinder)
- Alpha: 1.0 (fully opaque)
- Content: Zone name (e.g., "Zone_1")
```

### Troubleshooting

**Markers not appearing:**
1. Check MarkerArray display is added
2. Verify topic: `/delivery_zones/markers`
3. Check Fixed Frame is "map"
4. Ensure zone_marker_node is running: `ros2 node list | grep zone_marker`
5. Check markers are being published: `ros2 topic echo /delivery_zones/markers --once`

**Markers disappear:**
- They republish every second
- If node stops, markers will fade
- Restart zone definition mode to republish

## Testing

### Test Docking Line-of-Sight

```bash
# Scenario: Robot needs to navigate around obstacle to reach home
1. Place robot on opposite side of wall from home
2. Start delivery mode
3. Complete deliveries
4. Observe robot navigating around wall
5. Verify docking mode only activates when clear path exists
```

Expected logs:
```
[INFO] Returning home: 0.45m remaining
[INFO] Close to home (0.45m) but obstacle detected - continuing with path planner
[INFO] Returning home: 0.35m remaining
[INFO] Entering precise docking mode (0.35m from home, clear path)
[INFO] Docking: Moving forward (0.35m remaining)
```

### Test Zone Markers

```bash
# Test marker visibility
1. Start zone definition mode
2. Add MarkerArray display in RViz
3. Click 3-4 points on map
4. Verify colored cylinders appear
5. Verify text labels show zone names
6. Type 'clear' to test marker deletion
7. Verify all markers disappear
```

## Files Modified

- `src/DeliveryRobot.cpp` - Added line-of-sight check
- `include/DeliveryRobot.hpp` - Added hasLineOfSight method
- `src/zone_marker_node.cpp` - Immediate marker publishing, better logging
- `scripts/run_autonomous_slam.sh` - Added RViz setup instructions
