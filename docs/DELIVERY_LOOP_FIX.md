# Delivery Robot Fixes

## Problem 1: Infinite Loop at Zones

The robot was stuck in an infinite loop when trying to reach delivery zones:

```
[INFO] Found accessible point near Zone_1 at (0.51, -0.95)
[INFO] Path set with 3 waypoints, goal at (0.56, -0.50)
[INFO] Navigating to Zone_1
[INFO] Reached goal! Distance: 0.090m
[INFO] Found accessible point near Zone_1 at (0.51, -0.95)
[INFO] Path set with 3 waypoints, goal at (0.56, -0.50)
[INFO] Navigating to Zone_1
[INFO] Reached goal! Distance: 0.090m
... (repeats infinitely)
```

## Root Cause

1. **Zone position in obstacle**: The delivery zone at (0.5, -0.91) was inside an obstacle or too close to a wall
2. **Path planner adjustment**: The A* planner couldn't reach the exact position, so it found a "nearest accessible point" at (0.51, -0.95)
3. **Already at goal**: The robot was already within 0.1m (DISTANCE_TOLERANCE) of that accessible point
4. **Immediate goal reached**: MotionController declared "Reached goal!" and cleared the path
5. **Loop restart**: DeliveryRobot saw no path and tried to plan again, repeating the cycle

## Solution

### Change 1: Better Error Handling for Unreachable Zones

Added logic to skip zones that are completely unreachable after searching for accessible points:

```cpp
// If we still can't find a path, skip this zone
if (!found_goal) {
    RCLCPP_ERROR(node_->get_logger(),
                "Failed to plan path to %s - zone unreachable, skipping...", 
                current_zone.name.c_str());
    
    // Save failed delivery record
    DeliveryRecord record;
    record.timestamp = getCurrentTimestamp();
    record.from_zone = current_delivery_index_ > 0 ? 
        optimized_route_[current_delivery_index_ - 1].name : "Start";
    record.to_zone = current_zone.name;
    record.distance_traveled = total_distance_;
    record.success = false;
    record.notes = "Zone unreachable - no valid path found";
    saveDeliveryRecord(record);
    
    // Move to next zone
    current_delivery_index_++;
    publishStatus("Skipped unreachable: " + current_zone.name);
    return;
}
```

### Change 2: Fixed Zone Completion Detection

Changed the update logic to properly handle path execution and completion:

**New Logic:**
```cpp
// 1. If we have a path, execute motion
if (motion_controller_->hasPath()) {
    motion_controller_->computeVelocityCommand(current_pose, *current_map);
    return;  // Continue following path
}

// 2. No path - check if we just finished navigating
double distance_to_zone = /* calculate distance */;

if (distance_to_zone < ZONE_REACHED_THRESHOLD * 2.0) {
    // We're close and have no path = just arrived
    // Mark as reached and move to next zone
}

// 3. No path and not at zone - plan a new path
if (!motion_controller_->hasPath()) {
    // Plan path to zone...
}
```

This prevents the robot from instantly marking zones as complete before even starting navigation.

### Change 3: Removed Throttling from Warning

Changed from `RCLCPP_WARN_THROTTLE` to `RCLCPP_WARN` so the message only appears once per zone attempt, not repeatedly.

## How to Fix Zone Positions

If you want zones at exact positions, you need to place them in accessible areas:

1. **Check in RViz**: Make sure the zone is not in a black (obstacle) area
2. **Use Publish Point tool**: Click on white (free) areas of the map
3. **Verify zones**: Check `delivery_zones.yaml` to see stored positions
4. **Re-save zones**: Use `./scripts/delivery_commands.sh save` after adjusting

## Testing

After rebuilding:
```bash
colcon build --packages-select warehouse_robot_system
./scripts/run_autonomous_slam.sh -preload
# Select option 1 for Delivery Mode
```

The robot should now:
- Navigate to accessible points near zones
- Complete deliveries without looping
- Skip truly unreachable zones with error logging
- Continue to the next delivery in the route

---

## Problem 2: Starting Without Map

The delivery robot was starting before SLAM Toolbox had loaded and published the map:

```
Waiting for SLAM to be ready...
⚠️  /map topic not publishing yet, but continuing...
Starting Delivery Robot node...
```

This caused the robot to have no map data and be unable to plan paths.

### Root Cause

1. **Insufficient wait time**: The script only waited 10 seconds for the map
2. **Localization mode delay**: SLAM Toolbox in localization mode takes longer to load the map file
3. **No validation**: The delivery robot started immediately without checking for map availability

### Solution

#### Change 1: Better Map Waiting in Script

Improved the waiting logic in `run_autonomous_slam.sh`:

```bash
# Wait for SLAM to be ready (check if /map topic is publishing)
echo "   Waiting for SLAM to be ready..."
WAIT_COUNT=0
MAX_WAIT=30
MAP_READY=false

while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
    if ros2 topic info /map > /dev/null 2>&1; then
        # Topic exists, now check if it's publishing
        if timeout 2s ros2 topic echo /map --once > /dev/null 2>&1; then
            MAP_READY=true
            echo "   ✅ Map is being published"
            break
        fi
    fi
    sleep 1
    WAIT_COUNT=$((WAIT_COUNT + 1))
    if [ $((WAIT_COUNT % 5)) -eq 0 ]; then
        echo "   Still waiting for map... ($WAIT_COUNT/${MAX_WAIT}s)"
    fi
done

if [ "$MAP_READY" = false ]; then
    echo "   ⚠️  WARNING: Map not ready after ${MAX_WAIT}s"
    echo "   The delivery robot may not work correctly without a map!"
    echo "   Check SLAM Toolbox logs: tail -f /tmp/slam_toolbox.log"
    sleep 2
fi
```

Changes:
- Increased timeout from 10s to 30s
- Actually checks if map data is being published (not just if topic exists)
- Provides progress updates every 5 seconds
- Clear warning if map is not ready

#### Change 2: Map Validation in Delivery Robot Node

Added map checking in `delivery_robot_node.cpp`:

```cpp
// Wait for map to be available before starting deliveries
RCLCPP_INFO(node->get_logger(), "Waiting for map to be available...");

// Auto-start deliveries after map is ready (check every second)
auto start_timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [&, node]() {
        static bool started = false;
        static int wait_count = 0;
        
        if (!started) {
            // Check if we have a valid map
            if (g_robot && g_robot->hasValidMap()) {
                RCLCPP_INFO(node->get_logger(), "Map is ready! Auto-starting deliveries...");
                g_robot->startDeliveries();
                started = true;
            } else {
                wait_count++;
                if (wait_count % 5 == 0) {
                    RCLCPP_INFO(node->get_logger(), "Still waiting for map... (%ds)", wait_count);
                }
                
                // Timeout after 30 seconds
                if (wait_count >= 30) {
                    RCLCPP_ERROR(node->get_logger(), 
                                "Timeout waiting for map! Cannot start deliveries.");
                    RCLCPP_ERROR(node->get_logger(), 
                                "Check that SLAM Toolbox is running and publishing /map");
                    started = true;  // Stop trying
                }
            }
        }
    });
```

Added `hasValidMap()` method to DeliveryRobot class that checks if SLAM has a valid map.

### Testing

After rebuilding:
```bash
colcon build --packages-select warehouse_robot_system
./scripts/run_autonomous_slam.sh -preload
# Select option 1 for Delivery Mode
```

You should now see:
```
✅ Map is being published
Starting Delivery Robot node...
Waiting for map to be available...
Map is ready! Auto-starting deliveries...
```

The robot will only start deliveries once it has confirmed the map is available.
