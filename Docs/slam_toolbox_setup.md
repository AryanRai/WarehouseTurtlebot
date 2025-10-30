# SLAM Toolbox Setup Guide

## Overview
Successfully migrated from Cartographer to SLAM Toolbox for better Nav2 integration and delivery robot functionality.

## What Was Created

### 1. Configuration Files
- `config/slam_toolbox_params.yaml` - Mapping mode configuration
- `config/slam_toolbox_localization_params.yaml` - Localization mode for delivery
- `config/slam_toolbox.rviz` - Pre-configured RViz layout

### 2. Launch Files
- `launch/slam_toolbox_mapping.launch.py` - Launches SLAM Toolbox in mapping mode
- `launch/slam_toolbox_localization.launch.py` - Launches SLAM Toolbox in localization mode

### 3. Scripts
- `scripts/run_slam_toolbox_mapping.sh` - Complete autonomous mapping system with SLAM Toolbox

## Usage

### Mapping Phase (Current)
```bash
# Start Gazebo first
./launch_mgen.sh

# In another terminal, run SLAM Toolbox mapping
./scripts/run_slam_toolbox_mapping.sh

# With web dashboard
./scripts/run_slam_toolbox_mapping.sh -web
```

### Saving the Map
SLAM Toolbox auto-saves periodically, but you can manually save:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/tmp/warehouse_map'}}"
```

This creates:
- `/tmp/warehouse_map.yaml` - Map metadata
- `/tmp/warehouse_map.pgm` - Map image

### Localization Phase (For Delivery Robot)
After mapping is complete, switch to localization mode:
```bash
ros2 launch warehouse_robot_system slam_toolbox_localization.launch.py \
  use_sim_time:=true \
  map_file:=/tmp/warehouse_map
```

## Key Differences from Cartographer

| Feature | Cartographer | SLAM Toolbox |
|---------|-------------|--------------|
| Mapping Quality | Excellent | Excellent |
| Localization Mode | No | Yes (built-in) |
| Nav2 Integration | Manual | Native |
| Loop Closure | Yes | Yes |
| Map Saving | Manual | Automatic |
| Active Development | Slowing | Active |

## Next Steps for Delivery Robot

1. **Complete mapping phase** - Let robot explore and build map
2. **Save map** - Use service call or auto-save
3. **Create delivery waypoint manager** - C++ node to collect RViz goals
4. **Implement TSP optimization** - Order waypoints efficiently
5. **Add delivery logging** - Save delivery records to disk
6. **Integrate Nav2** - Use navigation stack for autonomous delivery

## RViz Interactive Goal Selection

In RViz, use the **2D Nav Goal** tool (top toolbar) to:
1. Click on the map where you want deliveries
2. Your C++ node subscribes to `/goal_pose` topic
3. Queue up multiple waypoints
4. Execute deliveries in optimal order

## Troubleshooting

### SLAM Toolbox won't start
```bash
# Check if installed
ros2 pkg list | grep slam_toolbox

# Install if missing
sudo apt install ros-jazzy-slam-toolbox
```

### Map not appearing in RViz
- Check Fixed Frame is set to `map`
- Verify `/map` topic is publishing: `ros2 topic echo /map --once`
- Check SLAM Toolbox logs: `tail -f /tmp/slam_toolbox.log`

### Localization not working
- Ensure map file exists at specified path
- Check map file format (YAML + PGM)
- Verify robot's initial pose is roughly correct
