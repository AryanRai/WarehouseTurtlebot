# Emergency Safety Patches for TF2 Stalling

## Quick Fixes to Add Now

These patches add immediate safety without restructuring the system.

### Patch 1: TF2 Health Monitor

Add to `InspectionRobot.hpp`:

```cpp
// Add to private members section (around line 130)
rclcpp::Time last_valid_tf_time_;
bool tf_is_healthy_;
static constexpr double TF_TIMEOUT = 2.0;  // Stop if no TF for 2 seconds

// Add to private methods section
bool checkTFHealth();
```

Add to `InspectionRobot.cpp` constructor (around line 45):

```cpp
last_valid_tf_time_(node->now()),
tf_is_healthy_(true),
```

Add new function in `InspectionRobot.cpp`:

```cpp
bool InspectionRobot::checkTFHealth() {
    try {
        // Try to get current pose - this uses TF2
        auto current_pose = slam_controller_->getCurrentPose();
        last_valid_tf_time_ = node_->now();
        
        // If we were unhealthy, log recovery
        if (!tf_is_healthy_) {
            RCLCPP_INFO(node_->get_logger(), "✅ TF2 recovered - resuming operation");
            publishStatus("TF2 recovered");
            tf_is_healthy_ = true;
        }
        return true;
        
    } catch (const std::exception& e) {
        auto time_since_valid = (node_->now() - last_valid_tf_time_).seconds();
        
        if (time_since_valid > TF_TIMEOUT) {
            if (tf_is_healthy_) {
                RCLCPP_ERROR(node_->get_logger(), 
                    "⚠️⚠️⚠️ TF2 FAILURE DETECTED! ⚠️⚠️⚠️");
                RCLCPP_ERROR(node_->get_logger(), 
                    "No valid transforms for %.1f seconds - EMERGENCY STOP", 
                    time_since_valid);
                RCLCPP_ERROR(node_->get_logger(), 
                    "Robot will resume when TF2 recovers");
                
                // EMERGENCY STOP
                motion_controller_->stop();
                tf_is_healthy_ = false;
                publishStatus("⚠️ TF2 FAILURE - STOPPED");
            }
            return false;
        }
        
        // Within grace period - just warn
        if (time_since_valid > 0.5) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "TF2 unstable for %.1f seconds", time_since_valid);
        }
        return true;
    }
}
```

Modify `update()` function (around line 300):

```cpp
void InspectionRobot::update() {
    // SAFETY CHECK: Stop if TF2 is failing
    if (!checkTFHealth()) {
        // Wait for TF2 to recover
        return;
    }
    
    if (!is_inspecting_) {
        return;
    }
    
    // ... rest of existing code
}
```

### Patch 2: Basic Obstacle Avoidance

Add to `InspectionRobot.hpp`:

```cpp
// Add to private members
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
static constexpr double OBSTACLE_STOP_DISTANCE = 0.35;  // 35cm safety margin
static constexpr double OBSTACLE_WARN_DISTANCE = 0.50;  // 50cm warning

// Add to private methods
bool isPathClear();
void onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
```

Add to `InspectionRobot.cpp` includes (top of file):

```cpp
#include <sensor_msgs/msg/laser_scan.hpp>
```

Add to constructor (around line 70):

```cpp
// Subscribe to laser scan for obstacle detection
laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&InspectionRobot::onLaserScan, this, std::placeholders::_1));
```

Add new functions:

```cpp
void InspectionRobot::onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
}

bool InspectionRobot::isPathClear() {
    if (!latest_scan_ || latest_scan_->ranges.empty()) {
        // No scan data yet - assume clear but warn
        return true;
    }
    
    // Check front 90 degrees (±45°) for obstacles
    size_t total_points = latest_scan_->ranges.size();
    size_t center = total_points / 2;
    size_t check_range = total_points / 4;  // 90° total (quarter of 360°)
    
    float min_distance = std::numeric_limits<float>::max();
    size_t obstacle_count = 0;
    
    for (size_t i = center - check_range; i < center + check_range; i++) {
        float range = latest_scan_->ranges[i % total_points];
        
        // Ignore invalid readings
        if (std::isnan(range) || std::isinf(range) || range < 0.1) {
            continue;
        }
        
        if (range < min_distance) {
            min_distance = range;
        }
        
        if (range < OBSTACLE_STOP_DISTANCE) {
            obstacle_count++;
        }
    }
    
    // If multiple points detect obstacle, it's real (not noise)
    if (obstacle_count > 3) {
        RCLCPP_ERROR(node_->get_logger(), 
            "⚠️ OBSTACLE DETECTED at %.2fm - EMERGENCY STOP", min_distance);
        publishStatus("⚠️ Obstacle detected");
        return false;
    }
    
    // Warning zone
    if (min_distance < OBSTACLE_WARN_DISTANCE) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "⚠️ Close to obstacle: %.2fm", min_distance);
    }
    
    return true;
}
```

Modify `update()` function:

```cpp
void InspectionRobot::update() {
    // SAFETY CHECK 1: Stop if TF2 is failing
    if (!checkTFHealth()) {
        return;
    }
    
    // SAFETY CHECK 2: Stop if obstacle detected
    if (!isPathClear()) {
        motion_controller_->stop();
        // Don't return - let robot try to recover or wait
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        return;
    }
    
    if (!is_inspecting_) {
        return;
    }
    
    // ... rest of existing code
}
```

---

## Build and Test

### 1. Apply Patches

```bash
cd ~/MTRX3760_Project_2_Fixing/turtlebot3_ws

# Edit the files as shown above
# Or use the provided patch files

colcon build --packages-select warehouse_robot_system
source install/setup.bash
```

### 2. Test TF2 Recovery

```bash
# Start your system normally
./scripts/run_autonomous_slam.sh -nocamui

# In another terminal, simulate TF failure
pkill -STOP slam_toolbox

# Watch robot stop within 2 seconds
# Then resume SLAM
pkill -CONT slam_toolbox

# Watch robot resume automatically
```

### 3. Test Obstacle Avoidance

```bash
# Start inspection
# Place hand/object in front of robot
# Robot should stop at 35cm
# Remove obstacle
# Robot should resume
```

---

## Expected Behavior

### With TF2 Monitor

**Before**:
```
[Robot moving]
[TF2 fails silently]
[Robot goes white in RViz]
[Robot continues moving blindly]
[CRASH into wall]
```

**After**:
```
[Robot moving]
[TF2 fails]
⚠️⚠️⚠️ TF2 FAILURE DETECTED! ⚠️⚠️⚠️
No valid transforms for 2.1 seconds - EMERGENCY STOP
[Robot STOPS immediately]
[TF2 recovers]
✅ TF2 recovered - resuming operation
[Robot continues safely]
```

### With Obstacle Avoidance

**Before**:
```
[Robot navigating]
[Wall ahead]
[CRASH]
```

**After**:
```
[Robot navigating]
⚠️ Close to obstacle: 0.48m
⚠️ OBSTACLE DETECTED at 0.32m - EMERGENCY STOP
[Robot STOPS]
[Waits or replans]
```

---

## Additional Recommendations

### 1. Always Use Headless Mode

```bash
# Add to your workflow
./scripts/run_autonomous_slam.sh -nocamui
```

This eliminates GUI overhead over network.

### 2. Monitor Network Quality

```bash
# On laptop, check WiFi signal
watch -n 1 'iwconfig 2>&1 | grep -i quality'

# Ping TurtleBot
ping <TURTLEBOT_IP>
# Should be <10ms, <1% loss
```

### 3. Reduce Camera Rate

If still having issues, reduce camera FPS on TurtleBot:
```bash
ssh ubuntu@<TURTLEBOT_IP>
# Edit camera config to 15 FPS instead of 30 FPS
```

---

## Summary

These patches add two critical safety features:

1. **TF2 Health Monitor**: Stops robot when transforms fail, resumes when recovered
2. **Obstacle Avoidance**: Stops robot before hitting walls

Combined with `-nocamui` flag, this should prevent crashes while you work on the long-term solution (moving camera processing to TurtleBot).

**Apply these patches immediately for safety, then plan the camera migration for reliability!**
