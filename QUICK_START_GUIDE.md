# Quick Start Guide - Temporal Filtering System

## 🚀 Running the Enhanced Detection System

### Option 1: Using the Enhanced Script (Recommended)

```bash
./scripts/enhanced_camera_detection.sh
```

Then choose:
- **Option 1**: AprilTag Detection Only (with temporal filtering)
- **Option 4**: Headless Mode (no GUI, perfect for SSH)

### Option 2: Manual Launch

```bash
# Source the workspace
cd ~/MTRX3760_Project_2_Fixing/turtlebot3_ws
source install/setup.bash

# Run the detector
ros2 run warehouse_robot_system apriltag_detector_node
```

## 📊 What You Should See

### Temporal Tracking Messages

```
⏱️ Started tracking tag ID 1
⏳ Tag ID 1 tracking: 0.25s / 1.00s (frames: 8)
⏳ Tag ID 1 tracking: 0.50s / 1.00s (frames: 15)
⏳ Tag ID 1 tracking: 0.75s / 1.00s (frames: 23)

🏷️ APRILTAG 16h5 DETECTED:
   📍 ID: 1
   📐 Center: (438.5, 160.7) pixels
   📏 Size: 146.1 pixels (diagonal)
   🔄 Orientation: -0.8° (yaw)
   ✅ Quality: margin=175.3, hamming=0
```

### False Positive Filtering

```
⏱️ Started tracking tag ID 15
⏳ Tag ID 15 tracking: 0.15s / 1.00s (frames: 5)
🗑️ Removed tag ID 15 from tracking (not seen for 0.52s)
```

## 🎯 Expected Behavior

### ✅ Genuine Tags (Stable)
- Detected continuously
- Tracking progresses: 0.25s → 0.50s → 0.75s → 1.00s
- Published after 1 second
- Visualization shows green box with ID label

### ❌ False Positives (Flickering)
- Detected briefly
- Tracking starts but doesn't reach 1 second
- Removed from tracking
- Never published or visualized

## 🔧 Troubleshooting

### Not Seeing Tracking Messages?

**Problem**: You're running the old binary without temporal filtering

**Solution**:
```bash
# Stop the detector (Ctrl+C)

# Rebuild and source
cd ~/MTRX3760_Project_2_Fixing/turtlebot3_ws
colcon build --packages-select warehouse_robot_system
source install/setup.bash

# Run again
ros2 run warehouse_robot_system apriltag_detector_node
```

### Still Getting False Positives?

**Increase the thresholds** in `AprilTagDetector.hpp`:

```cpp
const double kMinDecisionMargin = 60.0;       // Increase from 45
const double kMinDetectionDuration = 2.0;     // Increase from 1.0
```

Then rebuild:
```bash
colcon build --packages-select warehouse_robot_system
```

### Missing Genuine Tags?

**Decrease the thresholds** in `AprilTagDetector.hpp`:

```cpp
const double kMinDecisionMargin = 30.0;       // Decrease from 45
const double kMinDetectionDuration = 0.5;     // Decrease from 1.0
const double kMaxTimeSinceLastSeen = 1.0;     // Increase from 0.5
```

## 📈 Performance Tips

### For Maximum Accuracy
- Use **Option 1** (GUI mode) to see visual feedback
- Hold tags steady for at least 1 second
- Ensure good lighting conditions
- Print tags clearly on white paper

### For Maximum Speed
- Use **Option 4** (Headless mode) for no GUI overhead
- Perfect for SSH without -X flag
- Still gets full temporal filtering benefits

## 🎮 Testing the System

### Test 1: Stable Tag Detection
1. Hold ID 1 tag steady in front of camera
2. Wait for tracking messages
3. After 1 second, should see detection published
4. ✅ Success: Tag detected and published

### Test 2: False Positive Filtering
1. Wave tag quickly or show briefly
2. Should see tracking start
3. Should see tracking removed before 1 second
4. ✅ Success: No detection published

### Test 3: Environmental Noise
1. Point camera at textured surfaces
2. May see brief tracking messages
3. Should see removals before 1 second
4. ✅ Success: No false positives published

## 📝 Key Parameters

Located in `AprilTagDetector.hpp`:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `kMinDecisionMargin` | 45.0 | Quality threshold (tuned for ID 1 tags) |
| `kMaxHammingDistance` | 0 | Bit error tolerance (0 = perfect only) |
| `kMinDetectionDuration` | 1.0 | Stability time in seconds |
| `kMaxTimeSinceLastSeen` | 0.5 | Max gap before reset (seconds) |

## 🎯 Success Criteria

Your system is working correctly if:

- ✅ You see "⏱️ Started tracking" messages
- ✅ You see "⏳ Tag tracking" progress updates
- ✅ Genuine tags publish after 1 second
- ✅ False positives are removed before 1 second
- ✅ Only stable tags appear in visualization
- ✅ Console shows quality metrics (margin, hamming)

## 🆘 Need Help?

Check the detailed documentation:
- `TEMPORAL_FILTERING_SUMMARY.md` - Implementation details
- `CAMERA_ENHANCEMENTS_SUMMARY.md` - Full system overview
- `scripts/enhanced_camera_detection.sh` - Launch script with all options
