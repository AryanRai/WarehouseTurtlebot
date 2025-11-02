# Camera Detection Enhancement Summary

## 🎯 All Requested Features Implemented

### ✅ Camera Feed in All Modes
- **Calibration Mode**: Shows live camera with AprilTag overlays, sampling regions, and HSV stats
- **Normal Detection Mode**: Shows live camera with AprilTag detection boxes and ID labels  
- **File Mode**: Saves annotated frames to `/tmp/` when GUI unavailable

### ✅ AprilTag Visualization Enhancements
- **Green bounding boxes** around detected AprilTags (16h5 family)
- **Tag ID labels** displayed above each tag with black background for readability
- **Real-time corner detection** for accurate tag boundary visualization
- **Center point markers** showing exact tag center coordinates

### ✅ False Positive Filtering (Latest Update)
To address the issue of false detections from environmental patterns, implemented multi-layer filtering:

**Quality Filtering:**
- **Decision Margin Threshold**: Minimum 45.0 (tuned for ID 1 tags with ~50 margin)
- **Hamming Distance Filter**: Maximum 0 (only accepts perfect bit matches, no errors)
- **Reduced Decimation**: Changed from 2.0 to 1.5 for better accuracy

**Temporal Filtering (NEW):**
- **Continuous Detection Required**: Tags must be detected continuously for 1.0 second before being published
- **Tracking System**: Monitors each tag ID over time with frame counting
- **Gap Detection**: Resets tracking if tag disappears for more than 0.5 seconds
- **Automatic Cleanup**: Removes stale tracking data for tags no longer visible

This two-stage approach eliminates flickering false positives while ensuring only stable, genuine AprilTags are detected and published.

### ✅ Adaptive Sampling Regions
- **Distance-based scaling**: Larger tags (closer to camera) = larger sampling regions
- **Minimum size protection**: Ensures sampling regions never too small for reliable detection
- **Scale factor calculation**: Based on tag diagonal size relative to 80px baseline
- **Color-coded regions**:
  - � e**ABOVE tag**: Green sampling (mould detection)
  - 🟦 **BELOW tag**: Blue sampling (water damage)  
  - 🟥 **LEFT/RIGHT**: Red sampling (blood damage)

### ✅ Enhanced Calibration Guidance
- **Color placement instructions**: Tells user exactly where to place colored objects
- **HSV value reporting**: Shows mean values with variance ranges
- **Region size display**: Shows adaptive sampling rectangle dimensions
- **Real-time feedback**: Updates as objects are moved around AprilTags

## 🔧 Technical Fixes Applied

### False Positive Reduction
- **Quality Metrics**: Added decision margin and hamming distance filtering
- **Threshold Tuning**: Set minimum decision margin to 80.0 for reliable detection
- **Perfect Match Requirement**: Only accept tags with 0 bit errors (hamming = 0)
- **Enhanced Logging**: Shows quality metrics (margin, hamming) for each detection

### ROS Environment Issues
- **Fixed apriltag_ros dependency**: Removed external launch files (detection now built-in)
- **Logging directory setup**: Added `/tmp/ros_logs` to prevent rcutils errors
- **Environment isolation**: Clean environment variables to avoid snap conflicts

### Code Enhancements
- **String conversion fix**: `std::to_string(detection.id)` instead of `std::string(detection.id)`
- **Empty format string warnings**: Replaced empty strings with descriptive messages
- **Memory management**: Proper image cloning and boundary checking

## 📊 Quality Metrics Explained

**Instant Quality Checks:**
- **Decision Margin**: Confidence score for tag detection (higher = better)
  - Values < 45 are rejected as likely false positives
  - Your ID 1 tags have margins around 50-53 (good quality)
  - Most false positives have margins < 10

- **Hamming Distance**: Number of bit errors in tag decoding
  - 0 = perfect match (required)
  - >0 = corrupted data (rejected)

**Temporal Stability Checks:**
- **Detection Duration**: How long a tag has been continuously visible
  - Must be ≥ 1.0 second before publishing
  - Prevents flickering false positives from being published
  
- **Tracking Status**: Real-time monitoring of each tag
  - "⏱️ Started tracking" - New tag detected, timer started
  - "⏳ Tag tracking: X.XXs / 1.00s" - Still stabilizing
  - "✅ Published" - Tag stable for 1+ second
  - "🔄 Reset tracking" - Gap detected, restarting timer
  - "🗑️ Removed from tracking" - Tag no longer visible

## 🎮 User Interface Improvements

### Keyboard Controls (Calibration Mode)
- **'s'**: Save calibration to `~/hsv_calibration.yaml`
- **'c'**: Print detailed HSV values and placement guidance to terminal
- **'q'**: Quit calibration and exit

### Terminal Output Enhancements
```
⏱️ Started tracking tag ID 1
⏳ Tag ID 1 tracking: 0.25s / 1.00s (frames: 8)
⏳ Tag ID 1 tracking: 0.50s / 1.00s (frames: 15)
⏳ Tag ID 1 tracking: 0.75s / 1.00s (frames: 23)

🏷️ APRILTAG 16h5 DETECTED:
   📍 ID: 1
   📐 Center: (320.5, 240.3) pixels
   📏 Size: 85.2 pixels (diagonal)
   🔄 Orientation: -12.5° (yaw)
   ✅ Quality: margin=175.3, hamming=0

🏷️ Tag ID 5 sampling regions:
  🟩 ABOVE (place GREEN here):  HSV [65±15, 180±50, 120±30]
  🟦 BELOW (place BLUE here):   HSV [110±10, 200±30, 140±25]
  🟥 LEFT  (place RED here):    HSV [5±8, 190±40, 130±35]
  🟥 RIGHT (place RED here):    HSV [175±5, 185±45, 125±40]

📋 NEXT STEPS:
1. Place colored objects around the AprilTag:
   • GREEN object ABOVE the tag (mould simulation)
   • BLUE object BELOW the tag (water simulation)  
   • RED objects LEFT and RIGHT of tag (blood simulation)
2. Watch the colored rectangles change in the camera window
3. When satisfied with detection, press 's' to save calibration
```

## 🚀 How to Use the Enhanced System

### 1. Connect to TurtleBot
```bash
ssh -X ubuntu@TURTLEBOT_IP
ros2 launch turtlebot3_bringup robot.launch.py
```

### 2. Start Calibration
```bash
./scripts/turtlebot_camera_calibration.sh
# Choose option 1 for Interactive Calibration
```

### 3. Calibration Process
1. **Position AprilTag**: Place 16h5 AprilTag in camera view
2. **Watch visualization**: Green box appears around detected tag with ID label
3. **Place colors**: Following the on-screen instructions:
   - 🟩 **Green object ABOVE** the tag (simulates mould)
   - 🟦 **Blue object BELOW** the tag (simulates water damage)
   - 🟥 **Red objects LEFT and RIGHT** (simulates blood damage)
4. **Monitor sampling**: Colored rectangles show adaptive sampling regions that scale with tag distance
5. **Check values**: Press 'c' to see HSV readings in terminal
6. **Save calibration**: Press 's' when satisfied with color detection

### 4. Test Detection
- Choose option 3 for Normal Detection Mode
- AprilTags will be detected with blue boundaries and ID labels
- Damage classification will be performed and published to `/warehouse/damage_reports`
- False positives will be automatically filtered out

## 🔧 Tuning Guide

**If you're still getting false positives:**
- Increase `kMinDecisionMargin` (currently 45.0) to 60.0 or higher in `AprilTagDetector.hpp`
- Increase `kMinDetectionDuration` (currently 1.0s) to 2.0s for even stricter filtering
- Keep `kMaxHammingDistance` at 0 for strictest filtering

**If you're missing genuine tags:**
- Decrease `kMinDecisionMargin` to 30.0 or lower
- Decrease `kMinDetectionDuration` to 0.5s for faster response
- Increase `kMaxHammingDistance` to 1 (allows 1 bit error)
- Increase `kMaxTimeSinceLastSeen` to 1.0s if tags briefly disappear
- Improve lighting conditions
- Ensure tags are printed clearly and not damaged

**Tunable Parameters in `AprilTagDetector.hpp`:**
```cpp
const double kMinDecisionMargin = 45.0;       // Quality threshold (tuned for ID 1 tags)
const int kMaxHammingDistance = 0;            // Bit error tolerance
const double kMinDetectionDuration = 1.0;     // Stability time (seconds)
const double kMaxTimeSinceLastSeen = 0.5;     // Max gap before reset (seconds)
```

## 📁 File Locations

### Generated Files
- **Calibration data**: `~/hsv_calibration.yaml`
- **Debug images**: `/tmp/calibration_frame_*.jpg` (when GUI unavailable)
- **Detection frames**: `/tmp/detection_frame_*.jpg` (fallback mode)

### Enhanced Scripts
- **Main script**: `./scripts/turtlebot_camera_calibration.sh` 
- **System status**: Option 4 in calibration menu

## 🔍 Adaptive Scaling Algorithm

The sampling regions now automatically scale based on tag distance:

```cpp
// Scale factor: larger tags (closer) = larger regions
double scale_factor = std::max(0.5, std::min(2.0, aTagSize / 80.0));

// Apply scaling to all parameters
int adaptive_offset = static_cast<int>(kSamplingOffset * scale_factor);
int adaptive_width = static_cast<int>(kSamplingWidth * scale_factor);
int adaptive_height = static_cast<int>(kSamplingHeight * scale_factor);
```

This ensures reliable color detection at various distances while maintaining proportional sampling relative to the tag size.

## ✅ All Requirements Met
- ✅ Camera feed visible in all modes
- ✅ AprilTag boxes drawn with ID labels  
- ✅ Calibration detects AprilTag position
- ✅ Adaptive sampling regions scale with tag distance
- ✅ Real-time HSV feedback and guidance
- ✅ Fixed environment and dependency issues
- ✅ Enhanced user interface with detailed instructions
- ✅ False positive filtering with quality metrics
- ✅ Automatic rejection of low-confidence detections

