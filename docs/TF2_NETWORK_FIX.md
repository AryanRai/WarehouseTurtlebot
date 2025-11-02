# TF2 Network Stalling Fix & Optimization

## Problem Analysis

You've identified a critical issue: **TF2 stalling causing robot to go white in RViz and crash into walls**

### Root Causes

1. **No Obstacle Avoidance** ❌ - Robot continues moving even when TF2 fails
2. **No TF2 Error Handling** ❌ - System doesn't detect or recover from TF2 failures
3. **Network Overload** ⚠️ - Running camera/AprilTag/SLAM over WiFi AP mode
4. **Processing Overload** ⚠️ - All vision processing on laptop over network

## Solutions

### 1. Add Emergency Stop on TF2 Failure ✅ (Immediate Fix)

Add TF2 health monitoring to stop robot when transforms fail.

### 2. Add Basic Obstacle Avoidance ✅ (Safety)

Use laser scan to stop before hitting walls.

### 3. Move Camera Processing to TurtleBot 🎯 (Best Solution)

Run AprilTag detection directly on TurtleBot to reduce network load.

### 4. Optimize Network Settings ⚡ (Quick Win)

Reduce bandwidth usage and improve reliability.

---

## Implementation

### Option A: Quick Safety Fixes (Recommended First)

These can be implemented immediately without restructuring:

#### 1. TF2 Health Monitor

Add to `InspectionRobot.cpp`:

```cpp
// In header file - add member variables
rclcpp::Time last_valid_tf_time_;
static constexpr double TF_TIMEOUT = 2.0;  // 2 seconds without TF = problem
bool tf_is_healthy_;

// In update() function - add TF health check
bool InspectionRobot::checkTFHealth() {
    try {
        auto current_pose = slam_controller_->getCurrentPose();
        last_valid_tf_time_ = node_->now();
        tf_is_healthy_ = true;
        return true;
    } catch (const std::exception& e) {
        auto time_since_valid = (node_->now() - last_valid_tf_time_).seconds();
        
        if (time_since_valid > TF_TIMEOUT) {
            if (tf_is_healthy_) {
                RCLCPP_ERROR(node_->get_logger(), 
                    "⚠️ TF2 FAILURE DETECTED! Stopping robot for safety.");
                RCLCPP_ERROR(node_->get_logger(), 
                    "No valid transforms for %.1f seconds", time_since_valid);
                
                // EMERGENCY STOP
                motion_controller_->stop();
                tf_is_healthy_ = false;
                publishStatus("TF2 FAILURE - STOPPED");
            }
            return false;
        }
        return true;  // Within timeout grace period
    }
}

// Call at start of update()
void InspectionRobot::update() {
    if (!checkTFHealth()) {
        // Wait for TF to recover
        return;
    }
    
    if (!is_inspecting_) {
        return;
    }
    // ... rest of update logic
}
```

#### 2. Basic Obstacle Avoidance

Add laser scan subscriber:

```cpp
// In header
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
static constexpr double OBSTACLE_STOP_DISTANCE = 0.3;  // 30cm

// In constructor
laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = msg;
    });

// Add obstacle check function
bool InspectionRobot::isPathClear() {
    if (!latest_scan_) return true;  // No scan data yet
    
    // Check front 60 degrees (±30°)
    size_t center = latest_scan_->ranges.size() / 2;
    size_t check_range = latest_scan_->ranges.size() / 6;  // 60° total
    
    for (size_t i = center - check_range; i < center + check_range; i++) {
        float range = latest_scan_->ranges[i];
        if (range < OBSTACLE_STOP_DISTANCE && range > 0.0) {
            RCLCPP_WARN(node_->get_logger(), 
                "⚠️ OBSTACLE DETECTED at %.2fm - STOPPING", range);
            return false;
        }
    }
    return true;
}

// Call in update() before moving
void InspectionRobot::update() {
    if (!checkTFHealth()) return;
    if (!isPathClear()) {
        motion_controller_->stop();
        publishStatus("Obstacle detected - stopped");
        return;
    }
    // ... rest of update
}
```

### Option B: Move Camera to TurtleBot (Best Long-term)

This is the **recommended solution** for your network issues.

#### Architecture Change

**Current (Problematic)**:
```
TurtleBot (WiFi AP) → Laptop
  - Raw camera images (HIGH bandwidth)
  - AprilTag processing on laptop
  - SLAM on laptop
  - All over WiFi
```

**Proposed (Optimized)**:
```
TurtleBot (WiFi AP)
  - Camera capture (local)
  - AprilTag detection (local)
  - Publish only detections (LOW bandwidth)
  
Laptop
  - SLAM Toolbox
  - Navigation
  - RViz
```

#### Benefits

1. **90% less network traffic** - Only detection results, not raw images
2. **More reliable TF2** - Less network congestion
3. **Faster detection** - No network latency
4. **Better performance** - Distributed processing

#### Implementation Steps

1. **Install dependencies on TurtleBot**:
```bash
ssh ubuntu@<TURTLEBOT_IP>
sudo apt install ros-jazzy-apriltag ros-jazzy-apriltag-msgs
sudo apt install libopencv-dev
```

2. **Copy detector to TurtleBot**:
```bash
# On laptop
cd ~/MTRX3760_Project_2_Fixing
scp -r turtlebot3_ws/src/turtlebot3_simulations/warehouse_robot_system ubuntu@<TURTLEBOT_IP>:~/
```

3. **Build on TurtleBot**:
```bash
ssh ubuntu@<TURTLEBOT_IP>
cd ~/warehouse_robot_system
colcon build --packages-select warehouse_robot_system
source install/setup.bash
```

4. **Create TurtleBot launch script**:
```bash
# On TurtleBot: ~/start_camera_detection.sh
#!/bin/bash
source ~/warehouse_robot_system/install/setup.bash
ros2 run warehouse_robot_system apriltag_detector_node \
    --ros-args -p show_visualization:=false -p print_detections:=true &
ros2 run warehouse_robot_system colour_detector_node &
```

5. **Update laptop script** to NOT start camera nodes:
```bash
# In run_autonomous_slam.sh - comment out camera node startup
# Don't start apriltag_detector_node on laptop
# Don't start colour_detector_node on laptop
```

### Option C: Network Optimization (Quick Wins)

#### 1. Reduce Camera Resolution

On TurtleBot, edit camera config:
```bash
ssh ubuntu@<TURTLEBOT_IP>
# Edit camera launch file to reduce resolution
# 640x480 → 320x240 (4x less data)
```

#### 2. Disable Camera Visualization

Already done with `-nocamui` flag, but ensure it's always used:
```bash
./scripts/run_autonomous_slam.sh -nocamui
```

#### 3. Increase ROS2 QoS Reliability

In AprilTag detector, use RELIABLE QoS:
```cpp
auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
```

#### 4. Use ROS2 Domain ID Isolation

Ensure only your robot's traffic:
```bash
export ROS_DOMAIN_ID=30  # Use unique ID
```

---

## Recommended Action Plan

### Phase 1: Immediate Safety (Today)
1. ✅ Add TF2 health monitoring
2. ✅ Add basic obstacle avoidance
3. ✅ Test with current setup

### Phase 2: Network Optimization (This Week)
1. ✅ Always use `-nocamui` flag
2. ✅ Reduce camera resolution
3. ✅ Test reliability improvement

### Phase 3: Architecture Change (Best Solution)
1. ✅ Move AprilTag detection to TurtleBot
2. ✅ Keep only SLAM on laptop
3. ✅ Enjoy reliable operation

---

## Testing TF2 Health

### Monitor TF2 Status

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Monitor transform rate
ros2 run tf2_ros tf2_monitor map base_footprint

# Check for TF errors
ros2 topic echo /tf --once
```

### Simulate TF Failure

To test your safety code:
```bash
# Kill SLAM Toolbox temporarily
pkill -STOP slam_toolbox

# Robot should stop within 2 seconds
# Then resume
pkill -CONT slam_toolbox
```

---

## Why This Happens

### TurtleBot AP Mode Limitations

1. **Limited Bandwidth**: WiFi AP mode ~50 Mbps shared
2. **High Latency**: 10-50ms typical, spikes to 200ms+
3. **Packet Loss**: 1-5% normal, worse with interference

### What's Using Bandwidth

| Component | Bandwidth | Impact |
|-----------|-----------|--------|
| Raw camera (640x480@30fps) | ~30 Mbps | 🔴 HIGH |
| Laser scan | ~1 Mbps | 🟢 LOW |
| TF2 transforms | ~0.5 Mbps | 🟢 LOW |
| SLAM map updates | ~2 Mbps | 🟡 MEDIUM |
| AprilTag detections only | ~0.1 Mbps | 🟢 LOW |

**Moving camera processing to TurtleBot eliminates the 30 Mbps camera stream!**

---

## Summary

### Your Questions Answered

1. **Obstacle avoidance?** ❌ No - needs to be added
2. **Fix TF2 error?** ✅ Yes - add health monitoring and stop on failure
3. **AP mode issue?** ✅ Yes - camera over WiFi is the bottleneck
4. **Move camera to TurtleBot?** ✅ **YES - This is the best solution!**

### Next Steps

1. **Immediate**: Add TF2 health check and obstacle avoidance (safety)
2. **Short-term**: Always use `-nocamui` and reduce camera resolution
3. **Best solution**: Move AprilTag detection to TurtleBot (eliminate network bottleneck)

The TF2 stalling is definitely caused by network overload from streaming camera data over WiFi AP mode. Moving the camera processing to the TurtleBot will solve this completely!
