# Return to Home Features Implementation

## Summary

Successfully implemented return-to-home functionality with multiple triggers and visualization.

## Features Implemented

### 1. Home Position Visualization in RViz
- Added static TF broadcaster publishing `home_position` frame at (0, 0) in map frame
- Visible in RViz by adding TF display and enabling `home_position` frame
- Launch file: `autonomous_slam.launch.py`

### 2. Return Home Service
- Service: `/return_home` (std_srvs/Trigger)
- Allows manual triggering of return home behavior
- Test with: `ros2 service call /return_home std_srvs/srv/Trigger`

### 3. Battery Monitoring & Auto Return
- Subscribes to `/battery/percentage` topic
- Automatically initiates return home when battery < 20%
- Displays warning in logs

### 4. Web UI Return Home Button
- Added "Return to Home" button in battery monitoring web interface
- Button calls `/return_home` service via rosbridge
- Shows low battery warning when < 20%
- Located in control panel next to battery info

### 5. Improved Home Detection
- Added `at_home_` state flag to prevent continuous replanning
- Threshold: 10cm from home position (0, 0)
- Sends explicit stop command when home is reached
- Clears path to prevent further movement

## Return Home Triggers

1. **Exploration Complete** ✅ (already implemented)
   - Automatically returns home when no more frontiers found
   
2. **Low Battery < 20%** ✅ (newly implemented)
   - Monitors `/battery/percentage` topic
   - Auto-triggers return home
   
3. **Manual Request** ✅ (newly implemented)
   - Via ROS service: `ros2 service call /return_home std_srvs/srv/Trigger`
   - Via web UI: Click "🏠 Return to Home" button

## Usage

### Important: Conda Environment
If you have Anaconda/Miniconda active, use the no-conda wrapper to avoid library conflicts:
```bash
./run_slam_no_conda.sh
```

Or manually deactivate conda first:
```bash
conda deactivate
./scripts/run_autonomous_slam.sh
```

### View Home Position in RViz
1. Launch autonomous SLAM: `./run_slam_no_conda.sh` (or `./scripts/run_autonomous_slam.sh` if conda is deactivated)
2. In RViz, add TF display
3. Enable `home_position` frame to see home location marker

### Manual Return Home (CLI)
```bash
ros2 service call /return_home std_srvs/srv/Trigger
```

### Manual Return Home (Web UI)
1. Open battery monitor: http://localhost:5173
2. Click "🏠 Return to Home" button
3. Confirmation alert will appear

### Test Low Battery Trigger
Publish low battery percentage:
```bash
ros2 topic pub /battery/percentage std_msgs/msg/Float32 "data: 15.0"
```

## Files Modified

- `autonomous_slam.launch.py` - Added home position TF broadcaster
- `autonomous_slam_node.cpp` - Added service and battery subscriber
- `AutonomousExplorationRobot.cpp` - Improved home detection logic
- `AutonomousExplorationRobot.hpp` - Added `at_home_` flag
- `App.jsx` - Added return home button and low battery warning
- `App.css` - Styled control panel and button
- `CMakeLists.txt` - Added std_srvs dependency
- `package.xml` - Added std_srvs dependency

## Action Interface (Future Enhancement)

Created `ReturnHome.action` interface for future action server implementation:
- Goal: trigger return home
- Feedback: distance remaining, estimated time, status
- Result: success, final distance, message

Currently using simpler service interface for immediate functionality.
