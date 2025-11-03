# Inspection Modes Separated

## Overview
Separated inspection into two distinct modes with different behaviors and speeds:

1. **Inspection Exploration** (Option 5) - Slow patrol to discover sites
2. **Inspection** (Option 6) - Fast navigation to known sites

## Mode Comparison

### Inspection Exploration Mode (Option 5)
**Purpose:** Discover and map damage sites

**Behavior:**
- Systematically patrols entire warehouse
- Stops at grid points
- Performs 360° scan (6 stops) at each point
- Automatically saves discovered AprilTags
- Returns home when complete

**Speeds:**
- Linear: 0.05 m/s (slow for stable localization)
- Angular: 0.6 rad/s (slow turns)
- 360° Scan: 0.3 rad/s (very slow)

**Use Case:** First-time mapping of damage locations

---

### Inspection Mode (Option 6)
**Purpose:** Visit and verify known damage sites

**Behavior:**
- Goes directly to pre-defined sites (like delivery)
- No patrolling
- Performs 360° scan (6 stops) at EACH site
- Logs AprilTag IDs
- Returns home when complete

**Speeds:**
- Linear: 0.10 m/s (faster - like delivery)
- Angular: 1.25 rad/s (faster turns)
- 360° Scan: 0.3 rad/s (slow for detection)

**Use Case:** Regular inspections of known locations

## Key Changes

### 1. Speed Profiles
```cpp
// Exploration Mode
motion_controller_->setInspectionSpeeds();  // 0.05 m/s, 0.6 rad/s

// Regular Inspection Mode  
motion_controller_->setDeliverySpeeds();    // 0.10 m/s, 1.25 rad/s
```

### 2. Navigation Behavior
**Before:** Both modes did slow patrol
**After:**
- Exploration: Slow patrol with grid coverage
- Inspection: Direct navigation to sites (like delivery)

### 3. AprilTag Detection
**Before:** Regular inspection just waited at site
**After:** Both modes perform 360° scan (6 stops, 1 sec each)

```cpp
// At each site/patrol point:
for (int i = 0; i < 6; i++) {
    // Rotate 60°
    // Stop
    // Pause 1 second for detection
}
```

## Performance Comparison

### Inspection Exploration
| Metric | Value |
|--------|-------|
| Speed | 0.05 m/s |
| Coverage | Full warehouse |
| Time | ~15-20 minutes |
| Purpose | Discovery |

### Inspection
| Metric | Value |
|--------|-------|
| Speed | 0.10 m/s (2x faster) |
| Coverage | Known sites only |
| Time | ~5-8 minutes |
| Purpose | Verification |

## Example Workflow

### Initial Setup (Day 1)
```bash
# Run exploration to discover sites
./scripts/run_autonomous_slam.sh
# Select Option 5: Inspection Exploration
# Robot discovers and saves sites to damage_sites.yaml
```

### Regular Inspections (Day 2+)
```bash
# Run inspection to verify known sites
./scripts/run_autonomous_slam.sh
# Select Option 6: Inspection Mode
# Robot visits known sites, verifies AprilTags
# Much faster than exploration
```

## Benefits

✅ **Exploration**: Slow and thorough for discovery  
✅ **Inspection**: Fast and efficient for verification  
✅ **Flexible**: Choose mode based on need  
✅ **Reliable**: 360° scans ensure detection  
✅ **Efficient**: Don't patrol when you know where sites are  

## Log Messages

### Exploration Mode
```
[INFO] Using INSPECTION speed profile (slow for AprilTag detection)
[INFO] Motion speeds set: linear=0.050 m/s, angular=0.600 rad/s
[INFO] 🔍 Mode: Inspection Exploration (Systematic Patrol)
[INFO] Generated 7 patrol points
[INFO] 🔄 Performing 360° scan for AprilTags (6 stops)...
```

### Inspection Mode
```
[INFO] Using DELIVERY speed profile (fast)
[INFO] Motion speeds set: linear=0.100 m/s, angular=1.250 rad/s
[INFO] 📋 Mode: Inspection (Visit Pre-defined Sites)
[INFO] Starting inspections for 3 sites
[INFO] ✓ Reached site: Damage_1 - Performing 360° scan...
[INFO] 🔄 Performing 360° scan for AprilTags (6 stops)...
```

## Files Modified
- `warehouse_robot_system/include/Robot/InspectionRobot.hpp`
- `warehouse_robot_system/src/Robot/InspectionRobot.cpp`

## Status
✅ Modes separated
✅ Exploration: Slow patrol (0.05 m/s)
✅ Inspection: Fast navigation (0.10 m/s)
✅ Both: 360° scans at sites
✅ Built and ready to test
