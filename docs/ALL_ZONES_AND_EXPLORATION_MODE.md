# All Zones Delivery and Exploration Mode

## Issue 1: Robot Only Visits 2 Zones

### Problem
Robot was hardcoded to only visit Zone_1 and Zone_2, even when 4+ zones were defined.

```cpp
// Old hardcoded approach
DeliveryRequest req1;
req1.from_zone = "Zone_1";
req1.to_zone = "Zone_2";

DeliveryRequest req2;
req2.from_zone = "Zone_2";
req2.to_zone = "Zone_1";
```

### Solution
Changed to dynamically generate delivery requests for ALL defined zones:

```cpp
// New dynamic approach
auto zones = g_robot->getZones();

for (size_t i = 0; i < zones.size(); i++) {
    DeliveryRequest req;
    req.from_zone = zones[i].name;
    req.to_zone = zones[(i + 1) % zones.size()].name;  // Wrap around
    g_robot->addDeliveryRequest(req);
}
```

### Delivery Pattern

Creates a round-trip through all zones:

**Example with 4 zones:**
```
Start → Zone_1 → Zone_2 → Zone_3 → Zone_4 → Zone_1 → Home
```

**Example with 6 zones:**
```
Start → Zone_1 → Zone_2 → Zone_3 → Zone_4 → Zone_5 → Zone_6 → Zone_1 → Home
```

### Benefits

1. **Scalable**: Works with any number of zones (2+)
2. **Complete Coverage**: Visits every defined zone
3. **Efficient Route**: TSP optimization still applies
4. **Automatic**: No code changes needed when adding zones

## Issue 2: No Way to Remap

### Problem
Once a map was created, there was no way to create a new map without manually restarting the system.

### Solution
Added "Exploration Mode" as first option in the menu.

### New Menu Structure

```
[1] 🗺️  EXPLORATION MODE
    • Autonomous frontier exploration
    • Create new map from scratch
    • SLAM mapping mode
    • Replaces existing map

[2] 📍 DEFINE DELIVERY ZONES
    • Click points in RViz to mark zones
    • Visualize zones on map
    • Save zones for delivery mode

[3] 📦 DELIVERY MODE
    • Multi-point delivery operations
    • Uses saved delivery zones
    • Route optimization (TSP)

[4] 🔍 INSPECTION MODE (Coming Soon)
[5] ❌ EXIT
```

### Exploration Mode Workflow

1. Select option 1: EXPLORATION MODE
2. Confirmation prompt (to prevent accidental map replacement)
3. System cleanly shuts down
4. Instructions to restart without -preload flag
5. New exploration begins

### Safety Features

**Confirmation Required:**
```
⚠️  WARNING: This will replace your existing map!

Are you sure you want to start exploration? (yes/no):
```

- Type "yes" to proceed
- Anything else cancels and returns to menu

### Usage Scenarios

**Scenario 1: First Time Setup**
```bash
./scripts/run_autonomous_slam.sh
# Automatically starts exploration
# Robot maps the warehouse
# Returns to menu when complete
```

**Scenario 2: Using Existing Map**
```bash
./scripts/run_autonomous_slam.sh -preload
# Loads existing map
# Shows mode selection menu
# Choose zones/delivery/inspection
```

**Scenario 3: Remapping**
```bash
./scripts/run_autonomous_slam.sh -preload
# Select option 1: EXPLORATION MODE
# Confirm: yes
# System restarts in exploration mode
# New map created
```

## Complete Workflow Example

### Initial Setup (Day 1)
```
1. ./scripts/run_autonomous_slam.sh
2. Robot explores and maps warehouse
3. Map saved as warehouse_map_final.yaml
4. Select option 2: DEFINE DELIVERY ZONES
5. Click 4 points in RViz
6. Press ENTER to save
7. Select option 3: DELIVERY MODE
8. Robot visits all 4 zones in round-trip
9. Returns home
```

### Daily Operations (Day 2+)
```
1. ./scripts/run_autonomous_slam.sh -preload
2. Select option 3: DELIVERY MODE
3. Robot visits all zones
4. Returns home
```

### Warehouse Layout Changed (Day 30)
```
1. ./scripts/run_autonomous_slam.sh -preload
2. Select option 1: EXPLORATION MODE
3. Confirm: yes
4. System restarts
5. Robot creates new map
6. Redefine delivery zones
7. Resume operations
```

## Technical Details

### Dynamic Zone Loading

Added `getZones()` method to DeliveryRobot:

```cpp
std::vector<DeliveryZone> getZones() const { return zones_; }
```

### Round-Trip Generation

Uses modulo arithmetic to wrap around:

```cpp
req.to_zone = zones[(i + 1) % zones.size()].name;
```

For 4 zones:
- i=0: Zone_1 → Zone_2 (0+1=1)
- i=1: Zone_2 → Zone_3 (1+1=2)
- i=2: Zone_3 → Zone_4 (2+1=3)
- i=3: Zone_4 → Zone_1 (3+1=4, 4%4=0)

### Exploration Mode Handler

Cleanly shuts down current processes and provides restart instructions:

```bash
# Cleanup current processes
cleanup

# Restart without preload flag
echo "Please run: ./scripts/run_autonomous_slam.sh"
exit 0
```

## Testing

### Test All Zones Delivery

```bash
# Define 5 zones
./scripts/run_autonomous_slam.sh -preload
# Select option 2: DEFINE DELIVERY ZONES
# Click 5 points
# Press ENTER
# Select option 3: DELIVERY MODE
# Verify robot visits all 5 zones
```

Expected log:
```
[INFO] Added 5 delivery requests for round-trip through all zones
[INFO] Starting deliveries with optimized route of 10 stops
[INFO] Navigating to Zone_1
[INFO] Reached zone: Zone_1
[INFO] Navigating to Zone_2
...
[INFO] Navigating to Zone_5
[INFO] Reached zone: Zone_5
[INFO] Navigating to Zone_1
[INFO] All deliveries completed! Returning home
```

### Test Exploration Mode

```bash
./scripts/run_autonomous_slam.sh -preload
# Select option 1: EXPLORATION MODE
# Type: yes
# System exits
./scripts/run_autonomous_slam.sh
# Robot starts exploring
# New map created
```

## Files Modified

- `src/delivery_robot_node.cpp` - Dynamic delivery request generation
- `include/DeliveryRobot.hpp` - Added getZones() method
- `scripts/run_autonomous_slam.sh` - Added exploration mode option
