# Warehouse World Setup - Quick Reference

## ✅ What's Been Created

### 1. **Gazebo World File**
- **Location**: `turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_shelves.world`
- **Specifications**:
  - Total area: 2.3m x 2.3m (matching your diagram)
  - 4 horizontal shelves (1.15m wide each)
  - Vertical spacing: 0.46m between shelves
  - Side aisles: 0.575m wide each
  - Robot spawns at origin (0, 0)
  - Gray perimeter walls, wooden-colored shelves

### 2. **Standalone Launch Script** (Optional)
- **Location**: `launch_warehouse.sh`
- **Usage**: `./launch_warehouse.sh`
- Direct launcher for warehouse world only

### 3. **Integrated with Maze Generator** ✨
- **Option 10** added to `launch_mgen.sh`
- Warehouse is now part of the main menu!

## 🚀 How to Use

### Launch from Main Menu:
```bash
./launch_mgen.sh
```

Then select:
```
10) Warehouse with Shelves (2.3m x 2.3m, 4 shelf units)
```

The script will:
1. Load the warehouse world in Gazebo
2. Spawn TurtleBot3 at origin (0, 0)
3. Display instructions for next steps

### After Warehouse Launches:

**Terminal 1** (already running): Gazebo with warehouse

**Terminal 2** - Start SLAM:
```bash
./scripts/run_slam.sh
```

**Terminal 3** - Start RViz:
```bash
./scripts/run_rviz.sh
```

**Terminal 4** - Run autonomous exploration:
```bash
./scripts/run_autonomous_slam.sh
```

## 📐 World Layout

```
        2.3m total width
    ┌─────────────────────┐
    │                     │
0.46│   ═════════════     │ 0.46m  ← Shelf 1 (1.15m wide)
    │                     │
0.46│   ═════════════     │ 0.46m  ← Shelf 2
    │                     │
0.46│   ═════════════     │ 0.46m  ← Shelf 3
    │                     │
0.46│   ═════════════     │ 0.46m  ← Shelf 4
    │                     │
    └─────────────────────┘
    ↑                   ↑
0.575m              0.575m
(left aisle)     (right aisle)

* Robot starts at center: (0, 0)
```

## 🎯 Testing the Warehouse

1. **Launch warehouse**: `./launch_mgen.sh` → option 10
2. **Verify in Gazebo**: 
   - 4 brown shelves in center
   - Gray walls around perimeter
   - TurtleBot3 at origin

3. **Test SLAM exploration**:
   - Robot should navigate around shelves
   - Map should show aisles and obstacles
   - Frontier exploration should find all areas

## 🛠 Customization

To modify the warehouse layout, edit:
```
turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_shelves.world
```

You can adjust:
- Shelf positions (change `<pose>` Y values)
- Shelf dimensions (change `<size>` in `<box>`)
- Wall colors (change `<ambient>` and `<diffuse>`)
- Add more shelves (copy existing shelf models)

## 📝 Notes

- World file uses SDF 1.6 format (Gazebo standard)
- All shelves are static (won't move)
- Physics engine: ODE (configured for TurtleBot3)
- Shelves have both collision and visual geometry
- Camera view pre-configured for top-down warehouse view

## 🔧 Troubleshooting

**If warehouse doesn't load:**
```bash
# Check if world file exists
ls -la turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/warehouse_shelves.world

# Rebuild workspace if needed
cd turtlebot3_ws
colcon build --packages-select turtlebot3_gazebo
source install/setup.bash
```

**If robot doesn't spawn:**
- Wait a few more seconds for Gazebo to fully initialize
- Check that TURTLEBOT3_MODEL=burger is set
- Verify workspace is sourced

---
**Ready to test!** Just run `./launch_mgen.sh` and select option 10.
