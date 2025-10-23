# Wall Following Control

## Current Status: DISABLED ❌

Wall following behavior has been disabled to allow for SLAM operation with manual teleop control.

## How to Re-enable Wall Following

To re-enable wall following behavior, modify the following file:

**File:** `turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src/CWallFollowingController.cpp`

**Line:** Around line 115 in the `compute_velocity` method

**Change:**
```cpp
// DISABLE WALL FOLLOWING FOR SLAM - Set to false to re-enable wall following
static const bool WALL_FOLLOWING_ENABLED = false;  // Change this to true
```

**To:**
```cpp
// DISABLE WALL FOLLOWING FOR SLAM - Set to false to re-enable wall following
static const bool WALL_FOLLOWING_ENABLED = true;   // Changed to true
```

Then rebuild the project:
```bash
./scripts/build_project.sh
```

## Wall Following States

When enabled, the robot operates in these states:

- **RIGHT_WALL_FOLLOW** - Following right wall using PID control
- **LEFT_WALL_FOLLOW** - Following left wall using PID control  
- **GO_STRAIGHT** - Moving straight toward red target
- **RECOVERY_BACKUP** - Backing up from obstacles
- **GOAL_REACHED** - Red target found and reached

## Current SLAM Configuration

With wall following disabled:
- Robot publishes zero velocity commands (stationary)
- Robot responds to external velocity commands (teleop)
- SLAM can operate normally with manual or autonomous control
- All sensor data still available for SLAM algorithms

## Testing

Test wall following status:
```bash
./scripts/test_wall_following_disabled.sh
```

Test SLAM operation:
```bash
./scripts/run_slam_sim.sh
```

## Configuration Parameters

Wall following behavior is controlled by these parameters in `CWallFollowingController::SConfiguration`:

- `mDesired_wall_distance` - Target distance from wall (0.5m)
- `mNominal_velocity` - Normal forward speed (0.20 m/s)
- `mMax_angular_velocity` - Maximum turn rate (0.6 rad/s)
- PID gains: `mPID_kp`, `mPID_ki`, `mPID_kd`

These parameters remain unchanged and will take effect when wall following is re-enabled.