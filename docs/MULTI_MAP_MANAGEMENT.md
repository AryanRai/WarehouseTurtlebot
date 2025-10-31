# Multi-Map Management System

## Overview

The warehouse robot system now supports saving, loading, and managing multiple maps. Each map includes:
- Map files (.yaml and .pgm)
- Delivery zones configuration
- Robot pose/orientation
- Description and metadata

## Features

✅ **Save Multiple Maps**: Store different warehouse layouts
✅ **Quick Loading**: Select map at startup
✅ **Zone Preservation**: Zones saved with each map
✅ **Map Library**: Organize maps by name
✅ **Easy Switching**: Change maps without losing data

## Usage

### 1. Save Current Map

**Option A: From Mode Selection Menu**
```bash
./scripts/run_autonomous_slam.sh -preload
# Select [4] SAVE CURRENT MAP
# Enter map name and description
```

**Option B: Using Map Manager**
```bash
./scripts/map_manager.sh save warehouse1 "Main warehouse layout"
```

### 2. List Saved Maps

```bash
./scripts/map_manager.sh list
```

Output:
```
📁 Available Maps:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

   [1] warehouse1
       ✓ Map file: warehouse1.yaml
       ✓ Zones: 5 delivery zones
       ℹ️  Main warehouse layout

   [2] warehouse2
       ✓ Map file: warehouse2.yaml
       ✓ Zones: 3 delivery zones
       ℹ️  Secondary storage area

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   Total: 2 map(s)
```

### 3. Load a Saved Map

**Option A: At Startup (Recommended)**
```bash
./scripts/run_autonomous_slam.sh -preload
# Select map from list
```

**Option B: Using Map Manager**
```bash
./scripts/map_manager.sh load warehouse1
# Then run: ./scripts/run_autonomous_slam.sh -preload
```

### 4. Delete a Map

```bash
./scripts/map_manager.sh delete old_map
# Confirm with 'yes'
```

## Map Storage Structure

Maps are stored in `turtlebot3_ws/saved_maps/`:

```
saved_maps/
├── warehouse1/
│   ├── warehouse1.yaml          # Map metadata
│   ├── warehouse1.pgm            # Map image
│   ├── warehouse1_pose.txt       # Robot pose
│   ├── delivery_zones.yaml       # Zone definitions
│   └── info.txt                  # Description & timestamp
├── warehouse2/
│   ├── warehouse2.yaml
│   ├── warehouse2.pgm
│   ├── warehouse2_pose.txt
│   ├── delivery_zones.yaml
│   └── info.txt
└── ...
```

## Workflow Examples

### Scenario 1: Multiple Warehouse Locations

```bash
# Map warehouse A
./scripts/run_autonomous_slam.sh
# Complete exploration
# Select [4] Save Map → "warehouse_a"

# Map warehouse B
./scripts/run_autonomous_slam.sh
# Complete exploration
# Select [4] Save Map → "warehouse_b"

# Later, work in warehouse A
./scripts/run_autonomous_slam.sh -preload
# Select warehouse_a from list
# Select [3] Delivery Mode
```

### Scenario 2: Testing Different Zone Configurations

```bash
# Create base map
./scripts/run_autonomous_slam.sh
# Complete exploration
# Define zones
# Save as "base_config"

# Try different zone layout
# Modify zones
# Save as "config_v2"

# Compare performance
./scripts/run_autonomous_slam.sh -preload
# Load "base_config" → test deliveries
# Load "config_v2" → test deliveries
```

### Scenario 3: Backup Before Changes

```bash
# Before making changes
./scripts/map_manager.sh save backup_$(date +%Y%m%d)

# Make changes (add zones, test, etc.)

# If something goes wrong
./scripts/map_manager.sh load backup_20251101
```

## Map Manager Commands

```bash
# List all maps
./scripts/map_manager.sh list
./scripts/map_manager.sh ls

# Save current map
./scripts/map_manager.sh save <name> [description]

# Load a map
./scripts/map_manager.sh load <name>

# Delete a map
./scripts/map_manager.sh delete <name>
./scripts/map_manager.sh rm <name>
```

## What Gets Saved

When you save a map, the following are preserved:

1. **Map Data**
   - Occupancy grid (.yaml)
   - Map image (.pgm)
   - Resolution and origin

2. **Delivery Zones**
   - Zone positions
   - Zone names
   - Zone descriptions

3. **Robot State**
   - Last known position
   - Orientation
   - For accurate relocalization

4. **Metadata**
   - Map name
   - Description
   - Creation timestamp

## What Doesn't Get Saved

- Active delivery queue
- Delivery history/logs
- RViz configuration
- Running processes

## Tips & Best Practices

### Naming Conventions
```bash
# Good names
warehouse_main
storage_area_2
test_layout_v3
backup_2025_11_01

# Avoid
map1  # Not descriptive
my map  # Spaces not allowed
test  # Too generic
```

### When to Save
- ✅ After completing exploration
- ✅ After defining all zones
- ✅ Before making major changes
- ✅ When switching to different warehouse
- ❌ During active exploration
- ❌ While robot is moving

### Organization
```bash
# Use descriptive names
./scripts/map_manager.sh save warehouse_floor1 "First floor layout"
./scripts/map_manager.sh save warehouse_floor2 "Second floor layout"

# Include version numbers
./scripts/map_manager.sh save config_v1 "Initial zone layout"
./scripts/map_manager.sh save config_v2 "Optimized zones"

# Date-based backups
./scripts/map_manager.sh save backup_$(date +%Y%m%d) "Daily backup"
```

## Troubleshooting

### Map Not Loading
```bash
# Check if map exists
./scripts/map_manager.sh list

# Verify map files
ls -la turtlebot3_ws/saved_maps/warehouse1/

# Try loading manually
./scripts/map_manager.sh load warehouse1
```

### Zones Not Appearing
- Ensure zones were defined before saving map
- Check if delivery_zones.yaml exists in map folder
- Reload map to refresh zones

### Map Selection Timeout
- Default timeout is 30 seconds
- Press 0 to use current/default map
- Or wait for timeout (uses default)

## Integration with Existing Workflow

The multi-map system integrates seamlessly:

1. **Exploration** → Save map with name
2. **Zone Definition** → Zones saved with map
3. **Delivery Mode** → Load map, zones auto-loaded
4. **TSP Optimization** → Works with any loaded map

## Future Enhancements

Planned features:
- Map comparison tool
- Automatic backups
- Map merging
- Cloud sync
- Map sharing between robots

---

Now you can manage multiple warehouse layouts efficiently!
