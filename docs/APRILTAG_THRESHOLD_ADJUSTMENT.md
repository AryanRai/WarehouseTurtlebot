# AprilTag Decision Margin Threshold Adjustment

## Problem
The decision margin threshold was set too high at 45.0, causing many valid AprilTag detections to be rejected:

```
[INFO] [apriltag_detector_node]: ❌ Rejected tag ID 1: low decision margin (43.0 < 45.0)
[INFO] [apriltag_detector_node]: ❌ Rejected tag ID 18: low decision margin (0.3 < 45.0)
[INFO] [apriltag_detector_node]: ❌ Rejected tag ID 12: low decision margin (0.9 < 45.0)
```

Tags with margins of 43.0 (very good quality) were being rejected unnecessarily.

## Solution
Lowered the decision margin threshold from **45.0 to 35.0**

### Decision Margin Explained
- **Decision margin** measures how confident the detector is that it found a real tag
- Higher values = more confident detection
- Typical good detections: 35-70
- Typical false positives: < 20

### Threshold Comparison
| Threshold | Effect |
|-----------|--------|
| 45.0 (old) | Very strict - rejects many valid tags |
| 35.0 (new) | Balanced - accepts good tags, rejects noise |
| 25.0 | Too loose - may accept false positives |

## Change Made

**File:** `AprilTagDetector.hpp`

```cpp
// Before
const double kMinDecisionMargin = 45.0;

// After  
const double kMinDecisionMargin = 35.0;
```

## Expected Results

### Before (45.0 threshold)
- Tag with margin 43.0 → ❌ Rejected
- Tag with margin 48.0 → ✅ Accepted
- Tag with margin 60.0 → ✅ Accepted

### After (35.0 threshold)
- Tag with margin 43.0 → ✅ Accepted
- Tag with margin 48.0 → ✅ Accepted
- Tag with margin 60.0 → ✅ Accepted
- Tag with margin 30.0 → ❌ Rejected (likely noise)

## Benefits

✅ **More detections**: Accept tags with margins 35-45 (previously rejected)  
✅ **Still reliable**: Threshold of 35 is still high enough to reject noise  
✅ **Better coverage**: Robot will detect more tags during patrol  
✅ **Balanced approach**: Not too strict, not too loose  

## Files Modified
- `turtlebot3_ws/src/turtlebot3_simulations/warehouse_robot_system/include/Camera/AprilTagDetector.hpp`

## Deployment
✅ Built on laptop
✅ Copied to TurtleBot at `~/camera_ws/install/turtlebot_camera/lib/turtlebot_camera/apriltag_detector_node`

## Testing
Restart the camera detection on TurtleBot and observe fewer rejections:

```bash
ssh ubuntu@10.42.0.1 'ROS_DOMAIN_ID=29 ~/turtlebot_start_camera.sh'
```

You should see more "🏷️ APRILTAG 16h5 DETECTED" messages and fewer "❌ Rejected" messages.

## Status
✅ Threshold adjusted from 45.0 → 35.0
✅ Built and deployed to TurtleBot
✅ Ready for testing
