# Temporal Filtering Implementation Summary

## Problem Solved
You were experiencing many false AprilTag detections from environmental patterns (IDs 0, 2, 3, 5, 6, 7, 12, 13, 15, 16, 18, 19, etc.) when only showing ID 1.

## Solution: Two-Stage Filtering

### Stage 1: Quality Filtering (Instant)
- **Decision Margin**: Minimum 150.0 (rejects low-confidence detections)
- **Hamming Distance**: Maximum 0 (only perfect bit matches)
- **Result**: Immediately rejects poor quality detections

### Stage 2: Temporal Filtering (1 Second)
- **Continuous Tracking**: Each tag ID is tracked over time
- **Stability Requirement**: Must be visible continuously for ≥ 1.0 second
- **Gap Detection**: Resets timer if tag disappears for > 0.5 seconds
- **Result**: Only stable, persistent tags are published

## How It Works

```
Frame 1:  Tag ID 15 detected → ⏱️ Started tracking
Frame 5:  Tag ID 15 still visible → ⏳ 0.25s / 1.00s
Frame 10: Tag ID 15 still visible → ⏳ 0.50s / 1.00s
Frame 15: Tag ID 15 still visible → ⏳ 0.75s / 1.00s
Frame 20: Tag ID 15 disappeared → 🗑️ Removed (flickering false positive)

Frame 1:  Tag ID 1 detected → ⏱️ Started tracking
Frame 30: Tag ID 1 still visible → ✅ Published! (stable genuine tag)
```

## Expected Behavior

**False Positives (Flickering):**
- Detected briefly
- Tracking starts
- Disappears before 1 second
- Never published ✅

**Genuine Tags (Stable):**
- Detected continuously
- Tracking accumulates
- Reaches 1 second threshold
- Published and visualized ✅

## Console Output Examples

```bash
# False positive - rejected quickly
⏱️ Started tracking tag ID 15
⏳ Tag ID 15 tracking: 0.25s / 1.00s (frames: 8)
🗑️ Removed tag ID 15 from tracking (not seen for 0.52s)

# Genuine tag - published after 1 second
⏱️ Started tracking tag ID 1
⏳ Tag ID 1 tracking: 0.25s / 1.00s (frames: 8)
⏳ Tag ID 1 tracking: 0.50s / 1.00s (frames: 15)
⏳ Tag ID 1 tracking: 0.75s / 1.00s (frames: 23)
🏷️ APRILTAG 16h5 DETECTED:
   📍 ID: 1
   📐 Center: (473.2, 222.5) pixels
   📏 Size: 113.9 pixels (diagonal)
   🔄 Orientation: -92.7° (yaw)
   ✅ Quality: margin=175.3, hamming=0
```

## Tunable Parameters

Located in `AprilTagDetector.hpp`:

```cpp
// Quality thresholds
const double kMinDecisionMargin = 150.0;      // Higher = stricter
const int kMaxHammingDistance = 0;            // 0 = perfect match only

// Temporal thresholds
const double kMinDetectionDuration = 1.0;     // Seconds of continuous visibility
const double kMaxTimeSinceLastSeen = 0.5;     // Max gap before reset
```

### Adjustment Guide

**Too many false positives still?**
- Increase `kMinDecisionMargin` to 200.0
- Increase `kMinDetectionDuration` to 2.0

**Missing genuine tags?**
- Decrease `kMinDecisionMargin` to 100.0
- Decrease `kMinDetectionDuration` to 0.5
- Increase `kMaxTimeSinceLastSeen` to 1.0

## Testing

1. **Start the detector:**
   ```bash
   source turtlebot3_ws/install/setup.bash
   ros2 run warehouse_robot_system apriltag_detector_node
   ```

2. **Observe the console:**
   - You should see tracking messages for all detected tags
   - False positives will show brief tracking then removal
   - Genuine tags will show progression to 1.0s then publication

3. **Verify behavior:**
   - Hold ID 1 tag steady → Should publish after 1 second
   - Wave tag quickly → Should not publish (unstable)
   - Environmental patterns → Should be rejected or not track long enough

## Benefits

✅ **Eliminates flickering false positives** - Brief detections never published
✅ **Ensures stable detections** - Only persistent tags make it through
✅ **Automatic cleanup** - No memory leaks from old tracking data
✅ **Informative logging** - Clear visibility into what's happening
✅ **Tunable thresholds** - Easy to adjust for your environment

## Files Modified

- `AprilTagDetector.hpp` - Added tracking struct and temporal constants
- `AprilTagDetector.cpp` - Implemented temporal filtering logic
- `CAMERA_ENHANCEMENTS_SUMMARY.md` - Updated documentation
