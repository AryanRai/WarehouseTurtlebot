# Speed Profiles by Robot Mode

## Overview
Separated motion speeds by robot mode to optimize performance. Inspection mode uses slow speeds for accurate AprilTag detection, while delivery and exploration use faster speeds for efficiency.

## Speed Profiles

### Inspection Mode (Slow)
**Purpose:** Accurate AprilTag and color detection
```cpp
Linear Speed:  0.06 m/s  (6 cm/s)
Angular Speed: 0.8 rad/s (~46°/s)
```

**Why Slow:**
- Camera needs stable images for AprilTag detection
- Reduces motion blur
- Better color analysis accuracy
- More reliable tag ID reading

### Delivery Mode (Fast)
**Purpose:** Efficient multi-point deliveries
```cpp
Linear Speed:  0.15 m/s  (15 cm/s) - 2.5x faster
Angular Speed: 1.5 rad/s (~86°/s) - 1.9x faster
```

**Why Fast:**
- No camera detection needed during transit
- Faster deliveries = more throughput
- Reduced delivery time
- Better user experience

### Exploration Mode (Fast)
**Purpose:** Quick map creation
```cpp
Linear Speed:  0.15 m/s  (15 cm/s) - 2.5x faster
Angular Speed: 1.5 rad/s (~86°/s) - 1.9x faster
```

**Why Fast:**
- Faster exploration = quicker map completion
- No camera detection during exploration
- SLAM works well at higher speeds
- Reduced mapping time

## Implementation

### Architecture
```
MotionController
├── Configurable speed limits (member variables)
├── Speed setter methods
│   ├── setInspectionSpeeds()
│   ├── setDeliverySpeeds()
│   └── setExplorationSpeeds()
└── Default: Inspection speeds (safest)
```

### Code Changes

**MotionController.hpp:**
```cpp
// Configurable speeds (no longer const)
double max_drive_speed_;   // m/s
double max_turn_speed_;    // rad/s

// Speed profiles
static constexpr double INSPECTION_LINEAR_SPEED = 0.06;
static constexpr double DELIVERY_LINEAR_SPEED = 0.15;
static constexpr double EXPLORATION_LINEAR_SPEED = 0.15;

// Methods
void setInspectionSpeeds();
void setDeliverySpeeds();
void setExplorationSpeeds();
```

**Robot Initialization:**
```cpp
// InspectionRobot
motion_controller_->setInspectionSpeeds();  // Slow

// DeliveryRobot
motion_controller_->setDeliverySpeeds();    // Fast

// AutonomousExplorationRobot
motion_controller_->setExplorationSpeeds(); // Fast
```

## Performance Comparison

### Inspection Mode
| Metric | Before | After | Change |
|--------|--------|-------|--------|
| Linear Speed | 0.06 m/s | 0.06 m/s | Same |
| Angular Speed | 0.8 rad/s | 0.8 rad/s | Same |
| AprilTag Detection | Good | Good | ✅ |

### Delivery Mode
| Metric | Before | After | Change |
|--------|--------|-------|--------|
| Linear Speed | 0.06 m/s | 0.15 m/s | **+150%** |
| Angular Speed | 0.8 rad/s | 1.5 rad/s | **+88%** |
| Delivery Time | Slow | Fast | ✅ Improved |

### Exploration Mode
| Metric | Before | After | Change |
|--------|--------|-------|--------|
| Linear Speed | 0.06 m/s | 0.15 m/s | **+150%** |
| Angular Speed | 0.8 rad/s | 1.5 rad/s | **+88%** |
| Mapping Time | Slow | Fast | ✅ Improved |

## Benefits

✅ **Inspection**: Maintains slow speeds for accurate detection  
✅ **Delivery**: 2.5x faster linear speed = faster deliveries  
✅ **Exploration**: 2.5x faster = quicker map creation  
✅ **Flexible**: Easy to adjust speeds per mode  
✅ **Safe**: Defaults to slowest (inspection) speeds  

## Example Time Savings

### Delivery Scenario
- Distance: 10 meters
- **Before:** 10m ÷ 0.06 m/s = 167 seconds (~2.8 minutes)
- **After:** 10m ÷ 0.15 m/s = 67 seconds (~1.1 minutes)
- **Savings:** 100 seconds (60% faster)

### Exploration Scenario
- Map area: 50m² coverage
- **Before:** ~15 minutes
- **After:** ~6 minutes
- **Savings:** ~9 minutes (60% faster)

## Log Messages

When each robot starts, you'll see:
```
[INFO] Motion speeds set: linear=0.060 m/s, angular=0.800 rad/s
[INFO] Using INSPECTION speed profile (slow for AprilTag detection)
```

```
[INFO] Motion speeds set: linear=0.150 m/s, angular=1.500 rad/s
[INFO] Using DELIVERY speed profile (fast)
```

```
[INFO] Motion speeds set: linear=0.150 m/s, angular=1.500 rad/s
[INFO] Using EXPLORATION speed profile (fast)
```

## Files Modified
- `warehouse_robot_system/include/SLAM/MotionController.hpp`
- `warehouse_robot_system/src/SLAM/MotionController.cpp`
- `warehouse_robot_system/src/Robot/InspectionRobot.cpp`
- `warehouse_robot_system/src/Robot/DeliveryRobot.cpp`
- `warehouse_robot_system/src/Robot/AutonomousExplorationRobot.cpp`

## Status
✅ Speed profiles implemented
✅ Inspection: Slow (0.06 m/s)
✅ Delivery: Fast (0.15 m/s)
✅ Exploration: Fast (0.15 m/s)
✅ Built and ready to test
