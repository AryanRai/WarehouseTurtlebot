# Zone Clear Feature

## Overview

Added ability to clear all delivery zones during zone definition mode by typing "clear" instead of pressing ENTER.

## Usage

### In Zone Definition Mode

When you see this prompt:

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Commands:
  • Press ENTER to finish and return to menu
  • Type 'clear' to delete all zones and start over
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
👉 
```

**Option 1: Finish**
- Press ENTER
- Saves current zones
- Returns to mode selection menu

**Option 2: Clear All Zones**
- Type `clear` and press ENTER
- Deletes all zones from file
- Clears all markers from RViz
- Allows you to start fresh
- Prompts again when done

## Implementation

### Script Changes

Modified `run_autonomous_slam.sh` to:
1. Accept user input instead of just ENTER
2. Check if input is "clear"
3. Call `/clear_zones` service if clear requested
4. Continue zone definition after clearing

### Zone Marker Node

Added `/clear_zones` service that:
1. Clears the zones vector
2. Saves empty zones file
3. Publishes DELETE markers to remove visualization
4. Returns success response

```cpp
void onClearZones(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    
    zones_.clear();
    saveZones();
    
    // Send delete markers
    visualization_msgs::msg::MarkerArray marker_array;
    for (int i = 0; i < 100; i++) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(delete_marker);
    }
    marker_pub_->publish(marker_array);
    
    response->success = true;
}
```

## Example Workflow

### Scenario: Made a Mistake

```
1. Click 5 points in RViz
2. Realize you clicked wrong locations
3. Type 'clear' at prompt
4. All markers disappear
5. Click new correct locations
6. Press ENTER to save
```

### Scenario: Starting Fresh

```
1. Run zone definition mode
2. See old zones from previous session
3. Type 'clear' to remove them
4. Define new zone layout
5. Press ENTER to save
```

## Technical Details

### Service Definition

**Topic:** `/clear_zones`  
**Type:** `std_srvs/srv/Trigger`  
**Request:** Empty  
**Response:**
- `success`: true if cleared
- `message`: "All zones cleared"

### Marker Deletion

Sends DELETE action for marker IDs 0-99:
- Covers up to 50 zones (2 markers per zone: cylinder + text)
- Ensures all markers are removed from RViz
- Immediate visual feedback

### File Operations

1. Clears zones vector in memory
2. Writes empty YAML file:
```yaml
delivery_zones: []
```
3. File remains valid for next load

## Benefits

1. **Quick Recovery**: Fix mistakes without restarting
2. **Clean Slate**: Start fresh without manual file deletion
3. **Visual Feedback**: Markers disappear immediately
4. **No Restart Needed**: Continue in same session
5. **Safe Operation**: Doesn't affect running system

## Testing

```bash
cd turtlebot3_ws
colcon build --packages-select warehouse_robot_system
source install/setup.bash
cd ..
./scripts/run_autonomous_slam.sh -preload
```

1. Select option 1: DEFINE DELIVERY ZONES
2. Click 3-4 points
3. Type `clear` at prompt
4. Verify markers disappear
5. Click new points
6. Press ENTER
7. Verify new zones saved

## Files Modified

- `scripts/run_autonomous_slam.sh` - Added clear command handling
- `src/zone_marker_node.cpp` - Added `/clear_zones` service
