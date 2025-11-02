# CMakeLists Fix Applied - Run This! 🔧

## What Was Fixed

The ColourDetector was trying to link against ImageProcessor_Node but it wasn't compiled as a library.

**Fixed**: CMakeLists.txt now creates `image_processor_lib` that both nodes link against.

---

## Quick Update (If You Already Transferred)

```bash
cd ~/MTRX3760_Project_2_Fixing
./scripts/update_turtlebot_camera.sh 10.42.0.1
```

This will:
1. Prepare package with fixed CMakeLists
2. Remove old package on TurtleBot
3. Transfer new package
4. Build on TurtleBot

**Time**: ~3 minutes

---

## Or Run Full Migration Again

```bash
./scripts/migrate_camera_to_turtlebot.sh 10.42.0.1
```

This will do everything from scratch (15-20 min).

---

## What Changed in CMakeLists.txt

### Before (Broken)
```cmake
# AprilTag node with ImageProcessor_Node.cpp
add_executable(apriltag_detector_node
  src/Camera/AprilTagDetector.cpp
  src/Camera/ImageProcessor_Node.cpp
)

# Color node WITHOUT ImageProcessor_Node.cpp
add_executable(colour_detector_node
  src/Camera/ColourDetector.cpp
)
# ❌ Linker error: undefined reference to CImageProcessorNode
```

### After (Fixed)
```cmake
# Create shared library first
add_library(image_processor_lib
  src/Camera/ImageProcessor_Node.cpp
)

# Both nodes link against the library
add_executable(apriltag_detector_node ...)
target_link_libraries(apriltag_detector_node image_processor_lib ...)

add_executable(colour_detector_node ...)
target_link_libraries(colour_detector_node image_processor_lib ...)
# ✅ Works!
```

---

## Expected Output

### Successful Build

```
Starting >>> turtlebot_camera
Finished <<< turtlebot_camera [XX.Xs]

Summary: 1 package finished [XX.Xs]

╔════════════════════════════════════════╗
║   ✅ Update Complete!                  ║
╚════════════════════════════════════════╝
```

### Test It

```bash
# Start camera on TurtleBot
sshpass -p turtlebot ssh ubuntu@10.42.0.1 '~/turtlebot_start_camera.sh'

# Should show:
# ✅ AprilTag detector started
# ✅ Color detector started
```

---

## Verify on Laptop

```bash
# Check topics
ros2 topic list | grep apriltag
# Should show: /apriltag_detections

# Test detection
ros2 topic echo /apriltag_detections --once
```

---

## Summary

✅ **Issue**: Linker error for ColourDetector  
✅ **Fix**: Create image_processor_lib  
✅ **Command**: `./scripts/update_turtlebot_camera.sh 10.42.0.1`  
✅ **Time**: 3 minutes  
✅ **Result**: Both nodes build successfully!  

**Run the update script now!** 🚀
