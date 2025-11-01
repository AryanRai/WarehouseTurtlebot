# Delivery Robot Implementation Plan

## Requirements
From assignment.md:
- Uses **LiDAR + odometry** (no camera)
- Delivers goods between locations
- Handles **multiple delivery requests**
- Saves delivery records to disk (text format)

## Architecture

### Phase 1: Mapping (DONE ✅)
- Autonomous SLAM with SLAM Toolbox
- Frontier-based exploration
- Map saved to `/tmp/warehouse_map.yaml`

### Phase 2: Delivery System (TODO)

#### 2.1 Waypoint Collection
**Method:** RViz 2D Nav Goal interactive selection
- User clicks delivery locations in RViz
- C++ node subscribes to `/goal_pose` topic
- Waypoints stored in queue with metadata

#### 2.2 Route Optimization
**Algorithm:** Traveling Salesman Problem (TSP)
- Greedy nearest-neighbor for simplicity
- Or use 2-opt optimization for better results
- Calculate optimal delivery order

#### 2.3 Navigation Execution
**Stack:** Nav2 (Navigation2)
- Load saved map in localization mode
- Use SLAM Toolbox for localization
- Send waypoints to Nav2 action server
- Monitor delivery progress

#### 2.4 Delivery Logging
**Format:** Text file with delivery records
```
Timestamp | Waypoint ID | Location (x,y) | Status | Duration
```

## Implementation Steps

### Step 1: Create Delivery Waypoint Manager Node
```cpp
class DeliveryWaypointManager : public rclcpp::Node {
  // Subscribe to /goal_pose
  // Store waypoints in vector
  // Publish waypoint list
  // Service to start delivery run
};
```

### Step 2: Implement TSP Optimizer
```cpp
class RouteOptimizer {
  // Calculate distances between waypoints
  // Find optimal order (greedy or 2-opt)
  // Return ordered waypoint list
};
```

### Step 3: Create Delivery Controller
```cpp
class DeliveryController : public rclcpp::Node {
  // Load waypoints from manager
  // Send goals to Nav2
  // Monitor navigation status
  // Log delivery completion
  // Handle failures/retries
};
```

### Step 4: Integrate with Nav2
- Install Nav2 packages
- Configure Nav2 parameters
- Create launch file for delivery mode
- Test navigation to single waypoint
- Test multi-waypoint delivery

### Step 5: Add Delivery Logging
- Create log file writer
- Record delivery events
- Save to disk in text format
- Include timestamps and status

## File Structure
```
warehouse_robot_system/
├── include/
│   ├── DeliveryWaypointManager.hpp
│   ├── RouteOptimizer.hpp
│   └── DeliveryController.hpp
├── src/
│   ├── DeliveryWaypointManager.cpp
│   ├── RouteOptimizer.cpp
│   └── DeliveryController.cpp
├── config/
│   ├── nav2_params.yaml
│   └── delivery_params.yaml
└── launch/
    └── delivery_mode.launch.py
```

## Testing Plan
1. Test waypoint collection in RViz
2. Test TSP optimization with 3-5 waypoints
3. Test single waypoint navigation
4. Test multi-waypoint delivery sequence
5. Verify delivery logging
6. Test failure recovery

## Demo Script
1. Show completed map from mapping phase
2. Click 4-5 delivery locations in RViz
3. System calculates optimal route
4. Robot autonomously visits each location
5. Show delivery log file with records
6. Demonstrate handling of new delivery request