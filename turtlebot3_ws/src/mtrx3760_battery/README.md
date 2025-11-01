# MTRX3760 Battery Monitoring System

Complete battery monitoring solution for TurtleBot3 with terminal and web interfaces.

## 📦 Package Contents

### Nodes

1. **battery_simulator_node** - Simulates battery drain in Gazebo
2. **battery_monitor_node** - Monitors real battery on physical robot
3. **battery_terminal_display** - Terminal-based battery status display

### Topics

- **Published:** `/battery_state` (sensor_msgs/BatteryState)
- **Subscribed:** `/cmd_vel` (for drain calculation in simulator)

## 🚀 Quick Start

### Terminal Display

The terminal display shows real-time battery status with color-coded warnings:

```bash
# Start battery simulator (for Gazebo)
ros2 run mtrx3760_battery battery_simulator_node

# Start terminal display in new window
ros2 run mtrx3760_battery battery_terminal_display
```

**Features:**
- ✅ Real-time updates
- ✅ Color-coded status (Green/Yellow/Red)
- ✅ Battery bar visualization
- ✅ Voltage and percentage display
- ✅ Timestamp

### Web Dashboard

Modern React-based web interface with live charts:

```bash
# Launch everything (rosbridge + web server)
./scripts/launch_battery_web.sh
```

Then open: http://localhost:3000

**Features:**
- ✅ Circular battery gauge
- ✅ Live history chart
- ✅ Detailed battery info
- ✅ Charging indicator
- ✅ Connection status
- ✅ Responsive design

## 🔧 Configuration

Edit `config/battery_params.yaml`:

```yaml
battery_monitor:
  ros__parameters:
    # Voltage thresholds
    voltage_full: 12.6
    voltage_low: 11.0
    voltage_critical: 10.8
    
    # Simulation settings
    use_simulation: true
    initial_battery_pct: 100.0
    drain_rate_per_meter: 0.5
    drain_rate_per_rotation: 0.1
    
    # Auto return-to-home
    auto_return_home_enabled: true
    auto_return_threshold_pct: 20.0
```

## 📊 Display Options

### 1. Terminal Display

```bash
# Basic usage
ros2 run mtrx3760_battery battery_terminal_display

# With custom config
ros2 run mtrx3760_battery battery_terminal_display \
  --ros-args --params-file config/battery_params.yaml

# Clear screen mode (full refresh)
ros2 run mtrx3760_battery battery_terminal_display \
  --ros-args -p clear_screen:=true
```

**Terminal Output:**
```
┌─────────────────────────────────────────────────────┐
│         TURTLEBOT3 BATTERY STATUS MONITOR          │
└─────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────┐
│ Time:        14:23:45                               │
│ Voltage:     11.8 V                                 │
│ Percentage:  75 %  ███████░░░                       │
│ Status:      HEALTHY ✓                              │
└─────────────────────────────────────────────────────┘
```

### 2. Web Dashboard

**Prerequisites:**
```bash
# Install rosbridge
sudo apt install ros-humble-rosbridge-server

# Install web dependencies
cd turtlebot3_ws/src/mtrx3760_battery/web
npm install
```

**Launch:**
```bash
./scripts/launch_battery_web.sh
```

**Manual Launch:**
```bash
# Terminal 1: Start rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Start battery node
ros2 run mtrx3760_battery battery_simulator_node

# Terminal 3: Start web server
cd turtlebot3_ws/src/mtrx3760_battery/web
npm run dev
```

## 🔌 Integration with Autonomous SLAM

The battery monitoring is automatically integrated into the autonomous SLAM system:

```bash
./scripts/run_autonomous_slam.sh
```

This will:
1. Start battery simulator (Gazebo) or monitor (physical robot)
2. Open battery terminal display in new window
3. Monitor battery and trigger auto-return-home at 20%

## 🧪 Testing

### Test Battery Simulator

```bash
# Terminal 1: Start simulator
ros2 run mtrx3760_battery battery_simulator_node

# Terminal 2: Check topic
ros2 topic echo /battery_state

# Terminal 3: Drive robot to see drain
ros2 run turtlebot3_teleop teleop_keyboard
```

### Test Terminal Display

```bash
# Start display
ros2 run mtrx3760_battery battery_terminal_display

# In another terminal, publish test data
ros2 topic pub /battery_state sensor_msgs/BatteryState \
  "{voltage: 11.5, percentage: 65.0}"
```

### Test Web Dashboard

1. Start rosbridge: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
2. Start simulator: `ros2 run mtrx3760_battery battery_simulator_node`
3. Open browser: http://localhost:3000
4. Drive robot and watch battery drain in real-time

## 📝 Implementation Details

### Battery Simulator

Simulates realistic battery drain based on:
- **Idle drain:** 0.01% per second
- **Movement drain:** 0.5% per meter traveled
- **Rotation drain:** 0.1% per full rotation

### Battery Monitor

For physical robot:
- Subscribes to `/battery_state` from OpenCR board
- Converts voltage to percentage using LiPo curve
- Calculates discharge rate over time window

### Terminal Display

- Uses ANSI color codes for status indication
- Updates in-place (no screen flicker)
- Unicode block characters for battery bar
- Configurable update rate (default: 1 Hz)

### Web Dashboard

- WebSocket connection via rosbridge
- React 18 with functional components
- SVG-based charts and gauges
- Real-time data streaming
- Responsive design for mobile/desktop

## 🐛 Troubleshooting

**Terminal display not showing:**
- Check if battery topic is publishing: `ros2 topic list | grep battery`
- Verify node is running: `ros2 node list`
- Check logs: `tail -f /tmp/battery_display.log`

**Web dashboard not connecting:**
- Ensure rosbridge is running: `ros2 node list | grep rosbridge`
- Check WebSocket port: default is 9090
- Verify battery topic: `ros2 topic echo /battery_state`
- Check browser console for errors

**Battery not draining in simulation:**
- Make sure robot is moving: `ros2 topic echo /cmd_vel`
- Check simulator is subscribed: `ros2 topic info /cmd_vel`
- Verify config parameters in battery_params.yaml

## 📚 Additional Resources

- [ROS 2 BatteryState Message](https://docs.ros2.org/latest/api/sensor_msgs/msg/BatteryState.html)
- [rosbridge Documentation](http://wiki.ros.org/rosbridge_suite)
- [React Documentation](https://react.dev/)
- [Vite Documentation](https://vitejs.dev/)

## 🎯 Future Enhancements

- [ ] Battery health estimation
- [ ] Predictive time-to-empty calculation
- [ ] Historical data logging to file
- [ ] Email/SMS alerts for critical battery
- [ ] Mobile app (React Native)
- [ ] Integration with fleet management
