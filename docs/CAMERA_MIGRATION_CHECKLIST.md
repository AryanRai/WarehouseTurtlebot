# Camera Migration Checklist - Quick Reference

## ✅ Step-by-Step Checklist

### On Laptop

- [ ] **1. Prepare package**
  ```bash
  cd ~/MTRX3760_Project_2_Fixing
  ./scripts/prepare_turtlebot_camera.sh
  ```

### On TurtleBot (SSH)

- [ ] **2. SSH into TurtleBot**
  ```bash
  ssh ubuntu@192.168.0.XXX
  ```

- [ ] **3. Install dependencies**
  ```bash
  sudo apt update
  sudo apt install -y ros-jazzy-apriltag ros-jazzy-apriltag-msgs
  sudo apt install -y libopencv-dev ros-jazzy-cv-bridge
  sudo apt install -y libyaml-cpp-dev
  ```

- [ ] **4. Create workspace**
  ```bash
  mkdir -p ~/camera_ws/src
  ```

### On Laptop

- [ ] **5. Transfer package**
  ```bash
  /tmp/turtlebot_camera/transfer_to_turtlebot.sh 192.168.0.XXX
  ```

- [ ] **6. Transfer startup script**
  ```bash
  scp ~/MTRX3760_Project_2_Fixing/turtlebot_start_camera.sh ubuntu@192.168.0.XXX:~/
  ```

### On TurtleBot (SSH)

- [ ] **7. Build package**
  ```bash
  cd ~/camera_ws
  colcon build --packages-select turtlebot_camera
  source install/setup.bash
  ```

- [ ] **8. Make script executable**
  ```bash
  chmod +x ~/turtlebot_start_camera.sh
  ```

- [ ] **9. Test detector**
  ```bash
  # Make sure robot.launch.py is running first!
  ros2 run turtlebot_camera apriltag_detector_node --ros-args -p show_visualization:=false
  # Press Ctrl+C if it works
  ```

### On Laptop

- [ ] **10. Update run_autonomous_slam.sh**
  - Comment out AprilTag detector startup
  - Comment out camera node startup
  - Comment out color detector startup
  - Add notes that they run on TurtleBot

- [ ] **11. Test new workflow**
  - TurtleBot Terminal 1: `ros2 launch turtlebot3_bringup robot.launch.py`
  - TurtleBot Terminal 2: `~/turtlebot_start_camera.sh`
  - Laptop: `./scripts/run_autonomous_slam.sh -nocamui`

- [ ] **12. Verify topics**
  ```bash
  ros2 topic echo /apriltag_detections --once
  ```

- [ ] **13. Check network usage**
  ```bash
  sudo iftop -i wlan0
  # Should see ~2 Mbps instead of ~30 Mbps
  ```

- [ ] **14. Monitor TF2 stability**
  ```bash
  ros2 run tf2_ros tf2_monitor map base_footprint
  # Should be stable, <20ms latency
  ```

---

## Quick Commands Reference

### TurtleBot IP
```bash
# Replace XXX with your TurtleBot's actual IP
TURTLEBOT_IP=192.168.0.XXX
```

### One-Line Setup (After dependencies installed)
```bash
# On Laptop
./scripts/prepare_turtlebot_camera.sh && \
/tmp/turtlebot_camera/transfer_to_turtlebot.sh $TURTLEBOT_IP && \
scp turtlebot_start_camera.sh ubuntu@$TURTLEBOT_IP:~/
```

### One-Line Build (On TurtleBot)
```bash
cd ~/camera_ws && \
colcon build --packages-select turtlebot_camera && \
source install/setup.bash && \
chmod +x ~/turtlebot_start_camera.sh
```

---

## Verification Checklist

- [ ] Package builds without errors on TurtleBot
- [ ] AprilTag detector starts on TurtleBot
- [ ] Topics visible on laptop
- [ ] Network bandwidth reduced (use iftop)
- [ ] TF2 stable (no stalling)
- [ ] Inspection works normally
- [ ] No crashes during operation

---

## Success Indicators

✅ **On TurtleBot**:
```
✅ AprilTag detector started (PID: XXXX)
✅ Color detector started (PID: XXXX)
```

✅ **On Laptop**:
```
ros2 topic hz /apriltag_detections
# Shows: average rate: X.XXX
```

✅ **Network**:
```
sudo iftop -i wlan0
# Shows: ~2 Mbps (down from ~30 Mbps)
```

✅ **TF2**:
```
ros2 run tf2_ros tf2_monitor map base_footprint
# Shows: Average delay: 0.01s (down from 0.1-0.2s)
```

---

## Time Estimate

- Setup & transfer: 10 minutes
- Build & test: 10 minutes
- Update scripts: 5 minutes
- Testing: 10 minutes
- **Total: ~35 minutes**

---

## Rollback Plan

If something goes wrong:

1. Stop camera on TurtleBot (Ctrl+C)
2. Uncomment camera nodes in `run_autonomous_slam.sh`
3. Restart with old workflow

---

## Files Created

- `scripts/prepare_turtlebot_camera.sh` - Prepares package
- `turtlebot_start_camera.sh` - Runs on TurtleBot
- `/tmp/turtlebot_camera/` - Package to transfer
- `MOVE_CAMERA_TO_TURTLEBOT.md` - Full guide

---

## Ready to Start?

```bash
cd ~/MTRX3760_Project_2_Fixing
./scripts/prepare_turtlebot_camera.sh
```

Then follow the checklist above! 🚀
