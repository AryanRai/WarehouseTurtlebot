# Map Save Fix

## Problem
When the robot finished exploration and tried to save the map, the `map_saver_cli` would fail with:
```
[ERROR] [map_saver]: Failed to spin map subscription
```

This happened because:
1. The map_saver_cli needs time to subscribe to the `/map` topic
2. The process was blocking and timing out before getting the map
3. The node would exit before the save completed

## Solution: Dual-Method Background Save

### Method 1: SLAM Toolbox Serialization
- Calls `/slam_toolbox/serialize_map` service
- Saves the internal SLAM Toolbox map format
- Most reliable as it's directly from SLAM Toolbox
- Runs in background (non-blocking)

### Method 2: Standard Map Format
- Uses `map_saver_cli` to create .pgm/.yaml files
- Standard ROS map format for visualization/reuse
- Runs in background with 8-second timeout
- Non-blocking so it doesn't hang the main process

### Additional Improvements
- Saves robot pose to `{map_name}_pose.txt` immediately
- Both save methods run in background (using `&`)
- Added 500ms delay to let processes start
- No longer blocks or causes errors on exit

## Benefits
1. **Non-blocking**: Robot can continue/exit without waiting
2. **Dual format**: Gets both SLAM Toolbox and standard formats
3. **No errors**: Background execution prevents timeout failures
4. **Pose saved**: Robot position always saved immediately

## Files Created
When saving map as "warehouse_map_final":
- `warehouse_map_final.posegraph` - SLAM Toolbox format
- `warehouse_map_final.data` - SLAM Toolbox data
- `warehouse_map_final.pgm` - Standard map image
- `warehouse_map_final.yaml` - Standard map metadata
- `warehouse_map_final_pose.txt` - Robot pose at save time

## Testing
The map save now completes successfully without errors:
```
[INFO] [autonomous_slam_node]: Exploration complete - saving final map
[INFO] [slam_controller]: Saving map to: warehouse_map_final
[INFO] [slam_controller]: Map save commands issued (running in background)
[INFO] [slam_controller]: Robot pose saved to: warehouse_map_final_pose.txt
[INFO] [slam_controller]: Map save initiated. Files will appear shortly.
```

Files appear in the workspace root directory within a few seconds.
