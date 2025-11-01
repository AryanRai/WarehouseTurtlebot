# Warehouse AprilTag Layout

## Warehouse Dimensions

- **Total Area**: 2.3m x 2.3m
- **Wall Height**: 0.5m
- **Shelves**: 2 horizontal units (1.15m wide each)
- **Aisles**: ~0.7m spacing between shelves

## AprilTag Positions (Top View)

```
                    NORTH (Y = 1.15m)
        ┌─────────────────────────────────────┐
        │                                     │
        │         🏷️ Tag 0 (0.5, 1.14)       │
        │                                     │
WEST    │  ┌─────────────────────────┐       │  EAST
(-1.15) │  │      Shelf 1 (Top)      │       │  (1.15)
        │  └─────────────────────────┘       │
        │                                     │
        │              🤖 (0,0)               │  🏷️ Tag 0
        │             Robot Start             │  (1.14, 0.5)
        │                                     │
        │  ┌─────────────────────────┐       │
        │  │    Shelf 3 (Bottom)     │       │
        │  └─────────────────────────┘       │
        │                                     │
        │      🏷️ Tag 0 (-0.5, -1.14)        │
        │                                     │
        └─────────────────────────────────────┘
                    SOUTH (Y = -1.15m)
```

## AprilTag Details

### Tag 0 - North Wall
- **Position**: (0.5, 1.14, 0.3)
- **Orientation**: Facing South (yaw = π)
- **Height**: 30cm above ground
- **Visible From**: Center aisle, north approach

### Tag 0 - East Wall  
- **Position**: (1.14, 0.5, 0.3)
- **Orientation**: Facing West (yaw = -π/2)
- **Height**: 30cm above ground
- **Visible From**: Right aisle, east approach

### Tag 0 - South Wall
- **Position**: (-0.5, -1.14, 0.3)
- **Orientation**: Facing North (yaw = 0)
- **Height**: 30cm above ground
- **Visible From**: Center aisle, south approach

## Camera Detection Range

```
        Robot Camera FOV (Approximate)
        
              ╱────────╲
             ╱          ╲
            ╱            ╲
           ╱              ╲
          ╱                ╲
         🤖 ← 0.5m - 2.0m → 🏷️
         
    Optimal detection: 0.8m - 1.5m
    Minimum distance: 0.5m
    Maximum distance: 2.0m
```

## Navigation Paths to Tags

### Path to North Tag
```
Start (0, 0) → Move North → (0.5, 0.8) → Approach → (0.5, 1.0)
                                                      ↓
                                                   Detect Tag
```

### Path to East Tag
```
Start (0, 0) → Move East → (0.8, 0.5) → Approach → (1.0, 0.5)
                                                     ↓
                                                  Detect Tag
```

### Path to South Tag
```
Start (0, 0) → Move South → (-0.5, -0.8) → Approach → (-0.5, -1.0)
                                                         ↓
                                                      Detect Tag
```

## Inspection Robot Workflow

1. **Exploration Phase** (Optional)
   - Robot explores warehouse
   - Discovers AprilTags automatically
   - Marks damage sites

2. **Manual Site Definition**
   - Use RViz "Publish Point" tool
   - Click near AprilTag locations
   - Save sites: `./scripts/inspection_commands.sh save`

3. **Inspection Phase**
   - Robot navigates to each site
   - Approaches within 0.3m
   - Reads AprilTag ID
   - Logs result to CSV

## Adding More Tags

### Recommended Positions

**West Wall Tags**:
```xml
<!-- West wall, center -->
<pose>-1.14 0 0.3 0 0 1.5708</pose>

<!-- West wall, north -->
<pose>-1.14 0.5 0.3 0 0 1.5708</pose>

<!-- West wall, south -->
<pose>-1.14 -0.5 0.3 0 0 1.5708</pose>
```

**On Shelves**:
```xml
<!-- Shelf 1, front face -->
<pose>0 0.39 0.4 0 0 0</pose>

<!-- Shelf 3, front face -->
<pose>0 -0.39 0.4 0 0 3.14159</pose>
```

**Corner Positions**:
```xml
<!-- Northeast corner -->
<pose>0.8 0.8 0.3 0 0 -2.356</pose>

<!-- Northwest corner -->
<pose>-0.8 0.8 0.3 0 0 2.356</pose>

<!-- Southeast corner -->
<pose>0.8 -0.8 0.3 0 0 -0.785</pose>

<!-- Southwest corner -->
<pose>-0.8 -0.8 0.3 0 0 0.785</pose>
```

## Testing Checklist

- [ ] Tags visible in Gazebo
- [ ] Camera topic publishing: `/camera/image_raw`
- [ ] AprilTag detector running
- [ ] Detections appearing on `/apriltag_detections`
- [ ] Robot can navigate to tag locations
- [ ] Tag IDs correctly identified
- [ ] Inspection logs created

## Coordinate System Reference

```
        +Y (North)
         ↑
         │
         │
-X ──────┼────── +X (East)
(West)   │
         │
         ↓
        -Y (South)

Origin (0,0) = Robot start position
Z-axis points up (height)
```

## Related Files

- **World File**: `turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_shelves.world`
- **AprilTag Models**: `turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models/apriltag_36h11_id*/`
- **Generation Script**: `scripts/create_apriltag_models.sh`
- **Tag Generator**: `scripts/generate_apriltag.py`

## Quick Commands

```bash
# Generate 10 different AprilTag models
./scripts/create_apriltag_models.sh 10

# Launch warehouse with tags
./launch_mgen.sh  # Select option 10

# Test detection
ros2 topic echo /apriltag_detections

# Start inspection mode
./scripts/run_autonomous_slam.sh  # Select [5]
```
