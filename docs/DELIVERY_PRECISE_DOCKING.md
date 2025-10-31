# Delivery Robot Precise Docking Implementation

## Overview

Added precise docking capability to the delivery robot, matching the functionality from the exploration robot. This allows the robot to reach the exact home position (0, 0) with 5cm accuracy.

## Features Implemented

### 1. Precise Docking Mode

When the robot gets within 50cm of home, it switches from path-following to precise docking:

```cpp
// Enter precise docking mode when close to home (within 50cm)
if (distance_to_home < DOCKING_DISTANCE) {
    if (!in_docking_mode_) {
        RCLCPP_INFO(node_->get_logger(), "Entering precise docking mode (%.3fm from home)", distance_to_home);
        in_docking_mode_ = true;
        motion_controller_->clearPath();  // Stop using path planner
    }
    
    // Use precise docking control
    preciseDocking(current_pose, distance_to_home);
    return;
}
```

### 2. Docking Control Algorithm

The `preciseDocking()` method:

1. **Calculates angle to home**: Determines the direction to (0, 0)
2. **Aligns first**: If angle error > 10°, rotates in place
3. **Moves forward**: Once aligned, moves slowly forward with gentle angle correction
4. **Tight tolerance**: Stops when within 5cm of exact home position

```cpp
void DeliveryRobot::preciseDocking(const geometry_msgs::msg::Pose& current_pose, double distance_to_home) {
    // Calculate direction and angle to home
    double angle_to_home = std::atan2(dy, dx);
    double angle_diff = angle_to_home - yaw;
    
    // If angle is off by more than 10 degrees, rotate first
    if (std::abs(angle_diff) > 0.175) {
        cmd_vel.twist.linear.x = 0.0;
        cmd_vel.twist.angular.z = (angle_diff > 0) ? DOCKING_ANGULAR_SPEED : -DOCKING_ANGULAR_SPEED;
    } else {
        // Move forward slowly toward home
        cmd_vel.twist.linear.x = DOCKING_LINEAR_SPEED;
        cmd_vel.twist.angular.z = angle_diff * 0.5;  // Gentle correction
    }
}
```

### 3. Completion Detection

Improved home detection with tight tolerance:

```cpp
// Check if reached home with tight tolerance (5cm)
if (distance_to_home < HOME_TOLERANCE) {
    RCLCPP_INFO(node_->get_logger(), "✓ All deliveries completed! Robot at home (%.3fm).", distance_to_home);
    
    // Create completion marker file for script
    std::ofstream marker("/tmp/delivery_complete.marker");
    marker << "Delivery completed at " << getCurrentTimestamp() << std::endl;
    marker.close();
    
    stopDeliveries();
    return;
}
```

### 4. Return to Mode Selection

When deliveries complete, the robot:
1. Creates `/tmp/delivery_complete.marker` file
2. The launch script detects this marker
3. Stops the delivery robot node
4. Returns to mode selection menu
5. User can choose delivery mode again or inspection mode

## Parameters

```cpp
static constexpr double DOCKING_DISTANCE = 0.5;  // Enter docking mode within 50cm
static constexpr double HOME_TOLERANCE = 0.05;   // Success within 5cm
static constexpr double DOCKING_LINEAR_SPEED = 0.05;  // Slow speed (5cm/s)
static constexpr double DOCKING_ANGULAR_SPEED = 0.3;  // Moderate rotation (0.3 rad/s)
```

## Behavior Flow

### Normal Return Home (> 50cm from home)

1. Plan path to home using A* pathfinding
2. Follow path using Pure Pursuit controller
3. Monitor distance to home

### Docking Mode (< 50cm from home)

1. Clear path planner
2. Calculate direct angle to home
3. If misaligned (> 10°):
   - Rotate in place to face home
4. If aligned (< 10°):
   - Move forward slowly
   - Apply gentle angle correction
5. Stop when within 5cm

### Completion (< 5cm from home)

1. Log success message
2. Create completion marker file
3. Stop deliveries
4. Script returns to mode selection

## Testing

```bash
colcon build --packages-select warehouse_robot_system
./scripts/run_autonomous_slam.sh -preload
# Select option 1 for Delivery Mode
# Wait for deliveries to complete
```

Expected output:
```
[INFO] Returning home: 0.80m remaining
[INFO] Returning home: 0.45m remaining
[INFO] Entering precise docking mode (0.450m from home)
[INFO] Docking: Aligning (angle error: 15.3°)
[INFO] Docking: Moving forward (0.320m remaining)
[INFO] Docking: Moving forward (0.180m remaining)
[INFO] Docking: Moving forward (0.080m remaining)
[INFO] ✓ All deliveries completed! Robot at home (0.042m).
[INFO] Deliveries stopped

✅ Delivery system completed all tasks!
   Deliveries completed successfully!
   Returning to mode selection...
```

## Benefits

1. **Precise positioning**: Reaches exact (0, 0) within 5cm
2. **Smooth approach**: Slow, controlled docking prevents overshooting
3. **Reusable**: Can run multiple delivery sessions from same starting point
4. **User-friendly**: Automatically returns to menu for next operation
5. **Consistent**: Same proven algorithm as exploration robot

## Future Enhancements

Could add from exploration robot if needed:
- Obstacle detection during docking
- Recovery behaviors if docking fails
- Progress tracking and timeout detection
- Multiple docking attempts with different approaches
