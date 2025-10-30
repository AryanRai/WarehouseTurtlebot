# Return to Home Implementation Plan

## Overview
Implement a comprehensive "Return to Home" system for the autonomous robot.

## Home Position
- **Location**: (0, 0) - where the robot starts when SLAM begins
- **Visualization**: Marker in RViz showing home position
- **Frame**: map frame

## Triggers for Return to Home

### 1. Exploration Complete
- When no frontiers found after multiple recovery attempts
- Currently: Robot keeps doing recovery forever
- New: After N recovery attempts with no frontiers, return home

### 2. Low Battery
- Configurable threshold (default: 20%)
- Set via web interface
- Monitor battery level continuously
- Return home when below threshold

### 3. Manual Request
- From web interface (button)
- From RViz (interactive marker or service call)
- From terminal (ROS service call)

### 4. Delivery Request
- Before accepting delivery, return home first
- Ensures robot starts from known position

## Implementation Components

### A. ROS Action Server
- **Action**: `ReturnToHome.action`
- **Server**: Handles return-to-home requests
- **Feedback**: Distance to home, estimated time
- **Result**: Success/failure, final position

### B. Home Position Manager
- Store home position (0, 0)
- Publish home marker for RViz
- Provide service to get home position

### C. Battery Monitor Integration
- Add configurable threshold parameter
- Publish threshold to web interface
- Trigger return-to-home when low

### D. Web Interface Updates
- Battery threshold slider/input
- "Return to Home" button
- Display home position
- Show return-to-home status

### E. RViz Configuration
- Add home position marker
- Add interactive marker for manual return (optional)

## Files to Create/Modify

### New Files:
1. `action/ReturnToHome.action` - Action definition
2. `src/ReturnToHomeServer.cpp` - Action server implementation
3. `include/ReturnToHomeServer.hpp` - Header
4. `src/HomePositionManager.cpp` - Home position tracking
5. `include/HomePositionManager.hpp` - Header

### Modified Files:
1. `AutonomousExplorationRobot.cpp` - Add return-to-home logic
2. `AutonomousExplorationRobot.hpp` - Add home position tracking
3. `battery_monitor.cpp` - Add threshold checking
4. `web/src/App.jsx` - Add UI controls
5. `CMakeLists.txt` - Add action and new files
6. `package.xml` - Add action dependencies

## Implementation Steps

1. Create action definition
2. Implement home position manager
3. Implement return-to-home action server
4. Integrate with exploration (stop on complete)
5. Integrate with battery monitor
6. Add web interface controls
7. Update RViz configuration
8. Test all triggers

## Current Status
- ✅ Exploration working
- ✅ Recovery behavior working
- ⏳ Need to detect exploration complete
- ⏳ Need return-to-home implementation
- ⏳ Need battery threshold integration
- ⏳ Need web interface updates
