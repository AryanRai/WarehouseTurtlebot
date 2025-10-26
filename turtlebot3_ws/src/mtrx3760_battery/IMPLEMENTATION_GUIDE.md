# Battery Monitoring System - Implementation Guide

## Overview
This package provides battery monitoring for TurtleBot3 with terminal and GUI displays.

## Architecture

```
┌─────────────────────┐
│  Hardware/Gazebo    │
│  Battery Sensor     │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐      /battery_state topic
│  Battery Monitor    │──────────────┐
│      Node           │              │
└─────────────────────┘              │
                                     │
                    ┌────────────────┴───────────────┐
                    │                                │
                    ▼                                ▼
        ┌─────────────────────┐         ┌─────────────────────┐
        │ Terminal Display    │         │   GUI Display       │
        │      Node           │         │      Node           │
        └─────────────────────┘         └─────────────────────┘
```

## Implementation Steps

### Phase 1: Basic Monitoring (Start Here)

1. **Define Battery State Message**
   - Option A: Use `sensor_msgs/BatteryState` (standard ROS message)
   - Option B: Create custom message in `mtrx3760_interfaces`
   
   Recommended fields:
   ```
   float32 voltage          # Current voltage
   float32 percentage       # Battery percentage (0-100)
   float32 discharge_rate   # V/min or %/min
   float32 time_remaining   # Estimated minutes remaining
   bool is_charging         # Currently charging
   uint8 status            # GOOD=0, LOW=1, CRITICAL=2
   ```

2. **Implement Battery Monitor Node** (`battery_monitor_node.cpp`)
   
   For **Simulation** (easier to start):
   ```cpp
   - Initialize battery at 100%
   - Subscribe to /cmd_vel to track movement
   - Calculate drain: 
     * linear_drain = linear_velocity * dt * DRAIN_PER_METER
     * angular_drain = angular_velocity * dt * DRAIN_PER_ROTATION
   - Update battery_percentage -= (linear_drain + angular_drain)
   - Publish BatteryState at 1-5 Hz
   ```
   
   For **Real Robot**:
   ```cpp
   - Subscribe to /sensor_state (or similar topic from OpenCR)
   - Extract voltage from sensor data
   - Convert voltage to percentage using LiPo curve
   - Calculate discharge rate over time window
   - Publish BatteryState
   ```

3. **Implement Terminal Display** (`battery_terminal_display.cpp`)
   ```cpp
   - Subscribe to /battery_state
   - Format output with ANSI colors
   - Print battery bar visualization
   - Update terminal at 1 Hz
   ```

4. **Test Basic System**
   ```bash
   # Terminal 1: Launch monitor
   ros2 run mtrx3760_battery battery_monitor_node
   
   # Terminal 2: Launch display
   ros2 run mtrx3760_battery battery_terminal_display
   
   # Terminal 3: Drive robot and watch battery drain
   ros2 run turtlebot3_teleop teleop_keyboard
   ```

### Phase 2: GUI Display (Optional Enhancement)

5. **Choose GUI Framework**
   - **RQt Plugin**: Best for ROS 2 integration
   - **Standalone Qt**: Simpler to develop
   - **Web GUI**: Best for remote monitoring

6. **Implement GUI** (`battery_gui_display.cpp`)
   - Create battery gauge widget
   - Add voltage/percentage labels
   - Plot historical data (use QCustomPlot or similar)
   - Add warning banners

7. **Test GUI**
   ```bash
   ros2 run mtrx3760_battery battery_gui_display
   ```

### Phase 3: SLAM Integration (Advanced)

8. **Modify Autonomous SLAM Controller**
   
   Add to `autonomous_slam_controller.hpp`:
   ```cpp
   // Subscriber for battery state
   rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
   
   // Battery state
   sensor_msgs::msg::BatteryState current_battery_;
   bool low_battery_ = false;
   
   // Callback
   void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
   ```
   
   Add to `autonomous_slam_controller.cpp`:
   ```cpp
   // In executeStateMachine(), before MAPPING state:
   if (low_battery_ && current_state_ == SlamState::MAPPING) {
       RCLCPP_WARN(this->get_logger(), 
           "Low battery detected (%.1f%%), returning home", 
           current_battery_.percentage);
       transitionToState(SlamState::RETURNING_HOME);
       return;
   }
   
   // In batteryCallback():
   current_battery_ = *msg;
   low_battery_ = (msg->percentage < config_.auto_return_threshold_pct);
   ```

9. **Test Integration**
   - Run full autonomous SLAM
   - Verify robot returns home when battery low
   - Test with different thresholds

## File Guide

### Files to Implement (in order):

1. **START HERE**: `src/battery_monitor_node.cpp`
   - Core battery monitoring logic
   - Easiest with simulation mode

2. **NEXT**: `src/battery_terminal_display.cpp`
   - Visual feedback in terminal
   - Test battery monitor is working

3. **OPTIONAL**: `src/battery_gui_display.cpp`
   - Enhanced visualization
   - Skip if time constrained

4. **INTEGRATION**: Modify `autonomous_slam_controller.cpp`
   - Add battery awareness to SLAM
   - Auto return-to-home

### Configuration Files:

- `config/battery_params.yaml`: Tune thresholds and rates
- `launch/battery_monitoring.launch.py`: Launch all nodes together

## Code Examples

### Battery Monitor (Simulation Mode)
```cpp
class BatteryMonitor : public rclcpp::Node {
    double battery_pct_ = 100.0;
    rclcpp::Time last_update_;
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto now = this->now();
        double dt = (now - last_update_).seconds();
        
        // Calculate drain
        double linear_drain = std::abs(msg->linear.x) * dt * 0.5;  // 0.5% per m
        double angular_drain = std::abs(msg->angular.z) * dt * 0.1; // 0.1% per rad
        
        battery_pct_ -= (linear_drain + angular_drain);
        battery_pct_ = std::max(0.0, battery_pct_);
        
        last_update_ = now;
    }
};
```

### Terminal Display (ANSI Colors)
```cpp
std::string getBatteryBar(double pct) {
    int filled = static_cast<int>(pct / 10.0);
    std::string bar;
    for (int i = 0; i < 10; ++i) {
        bar += (i < filled) ? "█" : "░";
    }
    return bar;
}

void printBatteryStatus(const BatteryState& state) {
    std::string color = "\033[32m";  // Green
    if (state.percentage < 30.0) color = "\033[33m";  // Yellow
    if (state.percentage < 15.0) color = "\033[31m";  // Red
    
    std::cout << color 
              << "Battery: " << state.percentage << "% " 
              << getBatteryBar(state.percentage)
              << "\033[0m" << std::endl;
}
```

## Useful Resources

- ROS 2 BatteryState message: `sensor_msgs/msg/BatteryState`
- ANSI color codes: https://en.wikipedia.org/wiki/ANSI_escape_code
- RQt tutorial: https://docs.ros.org/en/humble/Tutorials/Beginner-GUI-Tools/Introducing-Rqt.html
- TurtleBot3 battery specs: 11.1V 1800mAh LiPo (3S)

## Testing Strategy

1. **Unit Test**: Monitor node with fake velocity commands
2. **Display Test**: Display node with fake battery messages
3. **Integration Test**: Full system with teleop
4. **SLAM Test**: Integration with autonomous exploration

## Next Steps

After reading this guide:
1. Start with `battery_monitor_node.cpp` in simulation mode
2. Use the code examples as starting point
3. Test with `battery_terminal_display.cpp`
4. Build and run: `colcon build --packages-select mtrx3760_battery`
5. If working well, integrate with SLAM controller
