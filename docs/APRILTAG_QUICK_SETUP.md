# AprilTag Quick Setup Guide

## 🚀 Quick Start (3 Steps)

### Step 1: Generate AprilTag Models
```bash
./scripts/create_apriltag_models.sh 5
```

### Step 2: Launch Warehouse with Tags
```bash
./launch_mgen.sh
# Select option 10 (Warehouse with Shelves)
```

### Step 3: Test Detection
```bash
# Terminal 1: Start inspection system
./scripts/run_autonomous_slam.sh
# Select [5] Inspection Mode

# Terminal 2: Check detections
ros2 topic echo /apriltag_detections
```

## ✅ What's Included

The warehouse world now has **3 AprilTags** (ID 0):
- **North Wall**: Position (0.5, 1.14, 0.3)
- **East Wall**: Position (1.14, 0.5, 0.3)  
- **South Wall**: Position (-0.5, -1.14, 0.3)

All tags are at **30cm height** (robot camera level) and **162mm x 162mm** size.

## 📝 Add More Tags

Edit `turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_shelves.world`:

```xml
<include>
  <uri>model://apriltag_36h11_id1</uri>
  <name>apriltag_custom</name>
  <pose>x y z 0 0 yaw</pose>
</include>
```

**Wall orientations (yaw in radians)**:
- North wall (facing south): `3.14159`
- South wall (facing north): `0`
- East wall (facing west): `-1.5708`
- West wall (facing east): `1.5708`

## 🔧 Troubleshooting

**Tags not visible?**
```bash
# Check models exist
ls turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/apriltag_*
```

**Tags not detected?**
```bash
# Check camera
ros2 topic hz /camera/image_raw

# Check detector
ros2 node list | grep apriltag
```

## 📚 Full Documentation

See [APRILTAG_GAZEBO_SETUP.md](APRILTAG_GAZEBO_SETUP.md) for complete details.
