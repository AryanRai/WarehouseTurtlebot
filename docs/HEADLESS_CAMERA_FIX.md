# Headless Camera Detection Fix

## Problem
The color detector was crashing when run over SSH (headless mode) with this error:
```
qt.qpa.xcb: could not connect to display
This application failed to start because no Qt platform plugin could be initialized.
[ros2run]: Aborted
```

## Root Cause
The color detector was calling OpenCV GUI functions (`cv::imshow()`, `cv::namedWindow()`, `cv::waitKey()`) even when running headless over SSH without a display. These functions require X11/Qt display support and crash when `DISPLAY` environment variable is not set.

## Solution
Added display availability checks before all OpenCV GUI calls:

1. **ProcessImage function** - Added `DISPLAY` check in calibration mode
2. **TagDetectionCallback function** - Added `DISPLAY` check before showing detection window  
3. **DisplayCalibrationWindow function** - Added `DISPLAY` check before creating windows

The fix checks `std::getenv("DISPLAY")` and only calls GUI functions if a display is available. When running headless, it logs a warning and skips GUI operations.

## Files Modified
- `turtlebot3_ws/src/turtlebot3_simulations/warehouse_robot_system/src/Camera/ColourDetector.cpp`

## Testing
Run the camera detection on TurtleBot over SSH:
```bash
ssh ubuntu@10.42.0.1 'ROS_DOMAIN_ID=29 ~/turtlebot_start_camera.sh'
```

The nodes should now run without crashing, even though there's no display available.

## Status
✅ Fixed and deployed to TurtleBot
✅ Color detector binary updated on TurtleBot
✅ Ready for testing
