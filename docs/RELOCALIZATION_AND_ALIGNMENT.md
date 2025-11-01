# Relocalization and Home Alignment Features

## Overview

Added three key improvements to both delivery and exploration robots for better positioning accuracy and repeatability:

1. **Initial Relocalization Spin** - 360° rotation before starting operations
2. **Return to Original Orientation** - Aligns back to starting angle when home
3. **Complete Stop** - Ensures all velocities are zero before completion

## Features

### 1. Initial Relocalization Spin

Before any movement, the robot performs a slow 360° spin to help SLAM accurately determine its position on the map.

**Parameters:**
```cpp
static constexpr double RELOCALIZATION_DURATION = 4.0;  // 4 seconds
static constexpr double RELOCALIZATION_SPEED = 0.5;     // 0.5 rad/s
```

**Behavior:**
- Stores initial orientation (yaw angle) at start
- Rotates in place for 4 seconds (full 360° at 0.5 rad/s)
- Stops completely before starting operations
- Logs progress every second

**Benefits:**
- Improves SLAM localization accuracy
- Helps robot understand its position on pre-loaded maps
- Reduces drift during operation
- Particularly useful in localization mode (delivery)

### 2. Return to Original Orientation

When the robot reaches home, it rotates to match its initial starting orientation.

**Tolerance:** 3 degrees (0.05 radians)

**Behavior:**
1. Robot reaches home position (< 5cm)
2. Calculates angle difference from initial orientation
3. Rotates slowly (0.2 rad/s) to align
4. Stops when within 3° of initial angle

**Benefits:**
- Consistent starting position for next operation
- Makes system more repeatable
- Robot always faces same direction at home
- Easier to predict behavior

### 3. Complete Stop

Ensures all motion is fully stopped before marking completion.

**Implementation:**
```cpp
// Stop all motion completely
geometry_msgs::msg::TwistStamped stop_cmd;
stop_cmd.header.stamp = node_->now();
stop_cmd.header.frame_id = "base_footprint";
stop_cmd.twist.linear.x = 0.0;
stop_cmd.twist.linear.y = 0.0;
stop_cmd.twist.linear.z = 0.0;
stop_cmd.twist.angular.x = 0.0;
stop_cmd.twist.angular.y = 0.0;
stop_cmd.twist.angular.z = 0.0;

recovery_cmd_vel_pub_->publish(stop_cmd);

// Give time for stop command to take effect
rclcpp::sleep_for(std::chrono::milliseconds(200));
```

**Benefits:**
- Prevents wheel drift after completion
- Ensures clean stop before mode switching
- Reduces wear on motors
- More professional appearance

## Implementation Details

### Delivery Robot

**Relocalization:**
- Happens at start of `update()` method
- Before any delivery operations begin
- Uses `/cmd_vel` topic for control

**Home Alignment:**
- Happens in `returnToHome()` method
- After reaching home position (< 5cm)
- Before creating completion marker file

### Exploration Robot

**Relocalization:**
- Happens at start of `update()` method
- Before any exploration begins
- Uses `recovery_cmd_vel_pub_` for control

**Home Alignment:**
- Happens in `returnToHome()` method
- After reaching home position (< 5cm)
- Before saving final map

## Sequence of Operations

### Delivery Mode Start:
```
1. Robot spawned at (0, 0)
2. Delivery node starts
3. Wait for map to be ready
4. Store initial orientation
5. Relocalization spin (4s, 360°)
6. Stop completely
7. Begin deliveries
```

### Delivery Mode End:
```
1. Complete all deliveries
2. Navigate home
3. Precise docking to (0, 0)
4. Check orientation
5. Rotate to match initial angle
6. Stop all motion
7. Wait 200ms
8. Create completion marker
9. Return to mode selection
```

### Exploration Mode Start:
```
1. Robot spawned at (0, 0)
2. Exploration node starts
3. Wait for SLAM initialization (15s)
4. Start exploration
5. Store initial orientation
6. Relocalization spin (4s, 360°)
7. Stop completely
8. Begin frontier exploration
```

### Exploration Mode End:
```
1. No more frontiers found
2. Navigate home
3. Precise docking to (0, 0)
4. Check orientation
5. Rotate to match initial angle
6. Stop all motion
7. Wait 200ms
8. Save final map
9. Create completion marker
10. Return to mode selection
```

## Testing

### Delivery Robot:
```bash
colcon build --packages-select warehouse_robot_system
./scripts/run_autonomous_slam.sh -preload
# Select option 1 for Delivery Mode
```

Expected output:
```
[INFO] Starting relocalization spin (360°)...
[INFO] Relocalizing... 3.0s remaining
[INFO] Relocalizing... 2.0s remaining
[INFO] Relocalizing... 1.0s remaining
[INFO] ✓ Relocalization complete! Starting deliveries...
...
[INFO] Docking: Moving forward (0.077m remaining)
[INFO] Aligning to initial orientation (5.3° remaining)
[INFO] Aligning to initial orientation (2.1° remaining)
[INFO] ✓ All deliveries completed! Robot at home (0.047m, aligned).
```

### Exploration Robot:
```bash
colcon build --packages-select warehouse_robot_system
./scripts/run_autonomous_slam.sh
```

Expected output:
```
[INFO] Starting relocalization spin (360°)...
[INFO] Relocalizing... 3.0s remaining
[INFO] ✓ Relocalization complete! Starting exploration...
...
[INFO] Docking: Moving forward (0.065m remaining)
[INFO] Aligning to initial orientation (4.7° remaining)
[INFO] ✓ Successfully returned to home position! (distance: 0.043m, aligned)
```

## Benefits Summary

1. **Better Localization**: Initial spin helps SLAM accurately determine position
2. **Repeatability**: Robot always returns to same position AND orientation
3. **Clean Stops**: No wheel drift or residual motion
4. **Professional**: Smooth, predictable behavior
5. **Mode Switching**: Consistent state for transitioning between modes
6. **Reduced Drift**: Better position tracking throughout operation

## Configuration

All parameters are configurable via constants in the header files:

**Relocalization:**
- `RELOCALIZATION_DURATION`: Time for full spin (default: 4.0s)
- `RELOCALIZATION_SPEED`: Rotation speed (default: 0.5 rad/s)

**Alignment:**
- Alignment tolerance: 0.05 rad (~3°)
- Alignment speed: 0.2 rad/s
- Stop delay: 200ms

**Home Position:**
- `HOME_TOLERANCE`: 0.05m (5cm)
- `DOCKING_DISTANCE`: 0.5m (50cm)
