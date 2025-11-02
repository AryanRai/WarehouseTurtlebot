# Temporal Filtering Toggle Guide

## Overview

You can now enable or disable temporal filtering using a ROS parameter while keeping all the code intact!

## Usage Options

### Option 1: WITH Temporal Filtering (Default: OFF)

**Pros:**
- ✅ Eliminates flickering false positives
- ✅ Only publishes stable detections (1 second continuous)
- ✅ Very reliable for stationary tags

**Cons:**
- ⏱️ 1-second delay before first detection
- 🔄 Resets if tag disappears briefly

**Use when:**
- You need maximum reliability
- False positives are a problem
- Tags are held steady

**Command:**
```bash
ros2 run warehouse_robot_system apriltag_detector_node \
    --ros-args -p enable_temporal_filtering:=true
```

**Or use script:**
```bash
./scripts/enhanced_camera_detection.sh
# Choose option 1
```

### Option 2: WITHOUT Temporal Filtering (Instant Detection)

**Pros:**
- ⚡ Instant detection - no delay
- 🚀 Faster response time
- ✅ Still has quality filtering (margin ≥ 45)

**Cons:**
- ⚠️ May see brief false positives
- 📊 More detections published

**Use when:**
- You need instant response
- Tags are moving quickly
- You can tolerate occasional false positives

**Command:**
```bash
ros2 run warehouse_robot_system apriltag_detector_node \
    --ros-args -p enable_temporal_filtering:=false
```

**Or use script:**
```bash
./scripts/enhanced_camera_detection.sh
# Choose option 2
```

## Comparison

| Feature | WITH Temporal | WITHOUT Temporal |
|---------|--------------|------------------|
| **Detection Speed** | 1 second delay | Instant |
| **False Positives** | Eliminated | Some may appear |
| **Quality Filter** | ✅ Active (≥45) | ✅ Active (≥45) |
| **Tracking Messages** | ✅ Shows progress | ❌ None |
| **Best For** | Stationary tags | Moving tags |
| **Reliability** | Very high | High |

## What Gets Filtered

### Quality Filter (Always Active)
- Decision margin < 45 → ❌ Rejected
- Hamming distance > 0 → ❌ Rejected

### Temporal Filter (When Enabled)
- Visible < 1 second → ⏳ Tracking (not published)
- Visible ≥ 1 second → ✅ Published
- Gap > 1 second → 🔄 Reset tracking

## Console Output Differences

### WITH Temporal Filtering
```
⏱️ Started tracking tag ID 1
⏳ Tag ID 1 tracking: 0.25s / 1.00s (frames: 8)
⏳ Tag ID 1 tracking: 0.50s / 1.00s (frames: 15)
⏳ Tag ID 1 tracking: 0.75s / 1.00s (frames: 23)

🏷️ APRILTAG 16h5 DETECTED:
   📍 ID: 1
   📐 Center: (475.4, 170.8) pixels
   📏 Size: 168.2 pixels (diagonal)
   🔄 Orientation: 0.0° (yaw)
   ✅ Quality: margin=54.9, hamming=0
```

### WITHOUT Temporal Filtering
```
🏷️ APRILTAG 16h5 DETECTED:
   📍 ID: 1
   📐 Center: (475.4, 170.8) pixels
   📏 Size: 168.2 pixels (diagonal)
   🔄 Orientation: 0.0° (yaw)
   ✅ Quality: margin=54.9, hamming=0

[Immediate detection - no tracking messages]
```

## Script Menu

The enhanced script now has 8 options:

1. **AprilTag WITH temporal filtering** - Maximum reliability
2. **AprilTag WITHOUT temporal filtering** - Instant detection ⚡ NEW!
3. **Color Calibration Mode** - HSV tuning
4. **Full Detection System** - AprilTag + Color
5. **HEADLESS Mode** - No GUI
6. **System Status** - Check processes
7. **Stop All Detectors** - Clean shutdown
8. **Exit** - Quit script

## Recommendation

**For your ID 1 tags:**

Since quality filtering (margin ≥ 45) already eliminates most false positives:

- **Use WITHOUT temporal filtering** (Option 2) for instant detection
- Quality filter still protects against false positives
- Faster response time
- Simpler operation

**Only use WITH temporal filtering** (Option 1) if:
- You're still seeing too many false positives
- You need absolute maximum reliability
- 1-second delay is acceptable

## Testing Both Modes

Try both and see which works better for your use case:

```bash
# Test instant mode
./scripts/enhanced_camera_detection.sh
# Choose option 2

# Test temporal mode
./scripts/enhanced_camera_detection.sh
# Choose option 1
```

## Parameters Summary

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enable_temporal_filtering` | false | Enable 1-second stability requirement |
| `show_visualization` | true | Show GUI window |
| `print_detections` | true | Print to console |

## Code Status

✅ All temporal filtering code is preserved
✅ Can be toggled with a single parameter
✅ No code changes needed to switch modes
✅ Both modes use the same quality filtering
