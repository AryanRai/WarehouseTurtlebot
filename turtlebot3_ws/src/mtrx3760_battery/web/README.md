# TurtleBot3 Battery Monitor - Web Interface

A modern, real-time web dashboard for monitoring TurtleBot3 battery status.

## Features

- 🔋 Real-time battery percentage gauge
- 📊 Live battery history chart
- 📈 Voltage, current, and temperature monitoring
- ⚡ Charging status indicator
- 🎨 Beautiful, responsive UI
- 🔌 WebSocket connection to ROS via rosbridge

## Prerequisites

1. **Node.js** (v16 or higher) - Check with `node --version`
2. **rosbridge_server** for ROS 2

Install rosbridge:
```bash
sudo apt install ros-humble-rosbridge-server
```

**Note:** This web app uses pure WebSocket connection to rosbridge - no native ROS bindings needed!

## Installation

```bash
cd turtlebot3_ws/src/mtrx3760_battery/web
npm install
```

## Usage

### 1. Start rosbridge server

In a terminal:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 2. Start battery simulator/monitor

In another terminal:
```bash
# For simulation
ros2 run mtrx3760_battery battery_simulator_node

# OR for physical robot
ros2 run mtrx3760_battery battery_monitor_node
```

### 3. Start the web interface

In another terminal:
```bash
cd turtlebot3_ws/src/mtrx3760_battery/web
npm run dev
```

### 4. Open in browser

Navigate to: http://localhost:3000

## Building for Production

```bash
npm run build
npm run preview
```

## Configuration

The web app connects to rosbridge at `ws://localhost:9090` by default.

To change this, modify the `wsUrl` in `src/App.jsx`.

## Troubleshooting

**Connection issues:**
- Make sure rosbridge is running
- Check that the battery topic `/battery_state` is publishing: `ros2 topic echo /battery_state`
- Verify rosbridge port (default: 9090)

**No data showing:**
- Ensure battery simulator or monitor node is running
- Check ROS 2 topics: `ros2 topic list`
- Verify battery_state messages: `ros2 topic echo /battery_state`

## Technology Stack

- React 18
- Vite (build tool)
- WebSocket (rosbridge connection)
- SVG (charts and gauges)
- CSS3 (styling)
