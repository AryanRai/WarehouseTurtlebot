# Tier 1 Safety - Quick Reference Card

## ✅ Status: IMPLEMENTED & READY TO TEST

---

## Start System

```bash
cd ~/MTRX3760_Project_2_Fixing/turtlebot3_ws
source install/setup.bash
cd ..
./scripts/run_autonomous_slam.sh -nocamui
```

---

## Test Safety Features

```bash
./scripts/test_tier1_safety.sh
```

---

## What's Protected

| Hazard | Protection | Action |
|--------|------------|--------|
| TF2 failure | Health monitor | Stop within 2s |
| Wall collision | Obstacle detection | Stop at 35cm |
| Network loss | TF2 timeout | Emergency stop |
| Sensor noise | 3-point filter | Ignore false positives |

---

## Safety Messages

### TF2 Failure
```
⚠️⚠️⚠️ TF2 FAILURE DETECTED! ⚠️⚠️⚠️
No valid transforms for 2.1 seconds - EMERGENCY STOP
```

### TF2 Recovery
```
✅ TF2 recovered - resuming operation
```

### Obstacle Detected
```
⚠️ OBSTACLE DETECTED at 0.32m - EMERGENCY STOP
```

### Warning Zone
```
⚠️ Close to obstacle: 0.48m
```

---

## Quick Tests

### Test TF2 Recovery (30 seconds)
```bash
pkill -STOP slam_toolbox   # Robot stops
pkill -CONT slam_toolbox    # Robot resumes
```

### Test Obstacle Avoidance (manual)
```
1. Start robot
2. Place hand in front (35cm)
3. Robot stops
4. Remove hand
5. Robot resumes
```

---

## Parameters

```cpp
TF_TIMEOUT = 2.0 seconds          // TF2 failure threshold
OBSTACLE_STOP_DISTANCE = 0.35m    // Emergency stop distance
OBSTACLE_WARN_DISTANCE = 0.50m    // Warning distance
Detection angle = 90° (±45°)      // Front coverage
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Stops too often | Network overloaded - apply Tier 2 |
| Doesn't stop | Check laser scan: `ros2 topic echo /scan` |
| False positives | Reduce OBSTACLE_STOP_DISTANCE to 0.30m |

---

## Next Steps

1. **Test** - Run `./scripts/test_tier1_safety.sh`
2. **Tier 2** - Reduce camera resolution/FPS
3. **Tier 3** - Move camera to TurtleBot (best solution)

---

## Files

- `TIER1_READY_TO_TEST.md` - Full testing guide
- `TIER1_IMPLEMENTATION_COMPLETE.md` - Technical details
- `scripts/test_tier1_safety.sh` - Testing tool
- `CAMERA_TO_TURTLEBOT_GUIDE.md` - Tier 3 guide

---

## Success Criteria

- [x] Code compiles ✅
- [ ] TF2 test passes
- [ ] Obstacle test passes
- [ ] No false positives
- [ ] Robot stops within 2s
- [ ] Robot stops at 35cm

---

**Ready to test! Start with `./scripts/test_tier1_safety.sh`**
