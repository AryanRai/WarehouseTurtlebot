# Quick Start Guide - Autonomous SLAM System

## Prerequisites

- ROS 2 Jazzy installed
- Gazebo Harmonic running
- TurtleBot3 packages installed
- Project built (`./scripts/build_project.sh`)

## Running the System

### Option 1: With Conda Active (Automatic Fix)

If you have Anaconda/Miniconda active in your terminal:

```bash
./run_slam_no_conda.sh
```

This automatically handles conda deactivation to prevent library conflicts.

### Option 2: Without Conda

If conda is not active or you've already deactivated it:

```bash
./scripts/run_autonomous_slam.sh
```

### Option 3: With Web Dashboard

To automatically start the battery monitoring web interface:

```bash
./run_slam_no_conda.sh -web
```

## What Gets Started

The system launches these components automatically:

1. **Robot State Publisher** - Publishes robot TF transforms
2. **TurtleBot3 in Gazebo** - Spawns robot at origin (0, 0)
3. **SLAM Toolbox** - Real-time mapping and localization
4. **ROSBridge WebSocket** - Enables web dashboard communication
5. **Battery Monitor** - Simulates and displays battery status
6. **RViz2** - Visualization with pre-configured SLAM view
7. **Autonomous SLAM Controller** - Frontier-based exploration

## Monitoring the System

### RViz Visualization

RViz opens automatically with:
- Map display (SLAM-generated map)
- LaserScan display (robot's sensor data)
- Path display (planned exploration paths)
- TF display (robot and home position frames)
- Robot model (TurtleBot3 visualization)

**Home Position Marker:**
- The `home_position` frame is visible at (0, 0) as colored axes
- Red = X, Green = Y, Blue = Z
- This shows where the robot will return when exploration completes

**Note:** If you see "Fixed Frame [map] does not exist" error initially, wait 10-20 seconds for SLAM to initialize.

For detailed RViz setup, see [RVIZ_HOME_POSITION.md](RVIZ_HOME_POSITION.md)

### Battery Web Dashboard

Open in your browser: http://localhost:5173

Features:
- Real-time battery percentage gauge
- Voltage, current, and temperature monitoring
- Historical data charts
- **🏠 Return to Home button** - Manual return home trigger
- Low battery warning (< 20%)

### Terminal Logs

- SLAM Toolbox: `tail -f /tmp/slam_toolbox.log`
- ROSBridge: `tail -f /tmp/rosbridge.log`
- RViz: `tail -f /tmp/rviz.log`
- Battery: Check the separate terminal window that opens

## Return to Home Features

The robot will automatically return home when:

1. **Exploration Complete** - No more frontiers to explore
2. **Low Battery** - Battery drops below 20%
3. **Manual Request** - Via web UI button or ROS service

### Manual Return Home

**Via Web UI:**
- Click the "🏠 Return to Home" button in the battery dashboard

**Via ROS Service:**
```bash
ros2 service call /return_home std_srvs/srv/Trigger
```

**Via Battery Simulation:**
```bash
# Trigger low battery return
ros2 topic pub /battery/percentage std_msgs/msg/Float32 "data: 15.0"
```

## Stopping the System

Press `Ctrl+C` in the terminal where you started the system. This will:
- Stop all ROS nodes gracefully
- Save the final map
- Clean up all processes

## Troubleshooting

### Library Conflicts (GLIBCXX errors)

**Problem:** Errors about missing GLIBCXX_3.4.32 or GLIBCXX_3.4.30

**Solution:** Use the no-conda wrapper:
```bash
./run_slam_no_conda.sh
```

See [CONDA_LIBRARY_FIX.md](CONDA_LIBRARY_FIX.md) for details.

### SLAM Not Starting

**Check:** Is Gazebo running?
```bash
gz sim --version
```

**Check:** Is the map topic available?
```bash
ros2 topic list | grep /map
```

### Robot Not Moving

**Check:** Is the autonomous controller running?
```bash
ros2 node list | grep autonomous_slam
```

**Check:** Are frontiers being detected?
```bash
ros2 topic echo /exploration/frontier_cells --once
```

### Web Dashboard Not Loading

**Check:** Is rosbridge running?
```bash
ros2 node list | grep rosbridge
```

**Check:** Is the web server running?
```bash
lsof -i :5173
```

**Manual start:**
```bash
cd turtlebot3_ws/src/mtrx3760_battery/web
npm install
npm run dev
```

## Advanced Usage

### Debug Mode

Enable debug visualization:
```bash
ros2 param set /autonomous_slam_node debug true
```

### Custom Map Save

Save map manually:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_custom_map'}}"
```

### View All Topics

```bash
ros2 topic list
```

Key topics:
- `/map` - SLAM-generated occupancy grid
- `/scan` - Laser scan data
- `/cmd_vel` - Robot velocity commands
- `/exploration/path` - Planned exploration path
- `/battery_state` - Battery status
- `/return_home` - Return home service

## Next Steps

- Monitor the robot's autonomous exploration in RViz
- Watch the map being built in real-time
- Test the return home features
- Experiment with different environments in Gazebo

For more details, see:
- [RETURN_HOME_FEATURES.md](RETURN_HOME_FEATURES.md) - Return home implementation
- [CONDA_LIBRARY_FIX.md](CONDA_LIBRARY_FIX.md) - Fixing conda conflicts
- [../README.md](../README.md) - Full project documentation
