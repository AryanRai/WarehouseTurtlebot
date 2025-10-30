# Return to Home - Quick Reference Card

## 🚀 Quick Start

```bash
# If conda is active (recommended)
./run_slam_no_conda.sh

# Or deactivate conda first
conda deactivate
./scripts/run_autonomous_slam.sh
```

## 🏠 Home Position in RViz

**What:** Colored axes at (0, 0) showing home location
**Where:** RViz TF display (enabled by default)
**Colors:** Red=X, Green=Y, Blue=Z

## 🔄 Three Ways to Return Home

### 1. Automatic (Exploration Complete)
Robot automatically returns when no more frontiers found.

### 2. Automatic (Low Battery)
Robot automatically returns when battery < 20%.

**Test it:**
```bash
ros2 topic pub /battery/percentage std_msgs/msg/Float32 "data: 15.0"
```

### 3. Manual (Your Choice)

**Option A - Web UI:**
1. Open http://localhost:5173
2. Click "🏠 Return to Home" button

**Option B - Command Line:**
```bash
ros2 service call /return_home std_srvs/srv/Trigger
```

## 🔧 Troubleshooting

### GLIBCXX Errors
```bash
./scripts/rebuild_clean.sh  # Run once
./run_slam_no_conda.sh      # Then use this
```

### Can't See Home Position
1. Check TF display is enabled in RViz
2. Wait 10-20 seconds for SLAM to start
3. Verify Fixed Frame is "map"

### Robot Won't Stop at Home
Already fixed! Robot stops within 10cm of home.

## 📚 Documentation

- Full features: `docs/RETURN_HOME_FEATURES.md`
- Quick start: `docs/QUICK_START.md`
- RViz setup: `docs/RVIZ_HOME_POSITION.md`
- Conda fix: `docs/CONDA_LIBRARY_FIX.md`
- Summary: `docs/IMPLEMENTATION_SUMMARY.md`

## ✅ Status

All features implemented and working:
- ✅ Home position visible in RViz
- ✅ Auto return on exploration complete
- ✅ Auto return on low battery
- ✅ Manual return via web UI
- ✅ Manual return via ROS service
- ✅ Proper stop at home (no infinite loop)
