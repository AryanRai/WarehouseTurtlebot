#!/bin/bash
# Autonomous SLAM System Runner
# Runs the complete autonomous SLAM system with frontier exploration
# Supports both Gazebo simulation and physical TurtleBot3
#
# Usage:
#   ./scripts/run_autonomous_slam.sh          # Normal mode
#   ./scripts/run_autonomous_slam.sh -web     # With web dashboard

# Deactivate conda if active to prevent library conflicts
if [ ! -z "$CONDA_PREFIX" ]; then
    echo "🔧 Deactivating conda environment..."
    echo "   Detected conda environment: $CONDA_DEFAULT_ENV"
    echo "   Deactivating for clean ROS environment..."
    echo ""
    
    # Unset all conda-related environment variables
    unset CONDA_PREFIX
    unset CONDA_DEFAULT_ENV
    unset CONDA_PROMPT_MODIFIER
    unset CONDA_SHLVL
    unset CONDA_PYTHON_EXE
    unset CONDA_EXE
    unset _CE_CONDA
    unset _CE_M
    
    # Remove conda from PATH
    export PATH=$(echo $PATH | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
    
    # Remove conda from LD_LIBRARY_PATH if it exists
    if [ ! -z "$LD_LIBRARY_PATH" ]; then
        export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
    fi
    
    echo "   ✅ Conda deactivated"
    echo ""
fi

# Parse command line arguments
START_WEB_DASHBOARD=false
PRELOAD_MAP=false
DISABLE_CAMERA_UI=false
BATTERY_THRESHOLD=-1.0  # Default: disabled (must be double for ROS parameter)
while [[ $# -gt 0 ]]; do
    case $1 in
        -web|--web)
            START_WEB_DASHBOARD=true
            shift
            ;;
        -preload|--preload)
            PRELOAD_MAP=true
            shift
            ;;
        -nocamui|--nocamui)
            DISABLE_CAMERA_UI=true
            shift
            ;;
        -battery|--battery)
            # Ensure it's a double by adding .0 if it's an integer
            if [[ "$2" =~ ^-?[0-9]+$ ]]; then
                BATTERY_THRESHOLD="$2.0"
            else
                BATTERY_THRESHOLD="$2"
            fi
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [-web] [-preload] [-nocamui] [-battery THRESHOLD]"
            echo ""
            echo "Options:"
            echo "  -web, --web              Start web dashboard (opens browser at http://localhost:3000)"
            echo "  -preload, --preload      Skip exploration, load existing map and go to mode selection"
            echo "  -nocamui, --nocamui      Disable camera viewfinder (detection still works)"
            echo "  -battery THRESHOLD       Enable low battery return (percentage, -1 to disable)"
            echo "                           Example: -battery 20 (return home at 20%)"
            echo "                                    -battery -1 (disable battery monitoring)"
            echo "  -h, --help               Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# Set ROS Domain ID
export ROS_DOMAIN_ID=29
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

if [ "$PRELOAD_MAP" = true ]; then
    echo "🗺️  Starting Warehouse System with Pre-loaded Map"
    echo "=================================================="
else
    echo "🤖 Starting Autonomous SLAM System with SLAM Toolbox"
    echo "====================================================="
fi
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
if [ "$START_WEB_DASHBOARD" = true ]; then
    echo "   Web Dashboard: ENABLED"
fi
if [ "$PRELOAD_MAP" = true ]; then
    echo "   Mode: PRELOAD (Skip exploration)"
fi
if [ "$DISABLE_CAMERA_UI" = true ]; then
    echo "   Camera UI: DISABLED (headless detection)"
fi
echo ""

# Save script directory before changing to workspace
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

cd "$(dirname "$0")/../turtlebot3_ws"

# Map selection for preload mode
if [ "$PRELOAD_MAP" = true ]; then
    MAPS_DIR="$(pwd)/saved_maps"
    
    # Check if there are saved maps
    if [ -d "$MAPS_DIR" ] && [ "$(ls -A $MAPS_DIR 2>/dev/null)" ]; then
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "📁 MAP SELECTION"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "   Available maps:"
        echo ""
        
        # List maps with numbers
        map_count=0
        declare -a map_names
        for map_dir in "$MAPS_DIR"/*; do
            if [ -d "$map_dir" ]; then
                map_count=$((map_count + 1))
                map_name=$(basename "$map_dir")
                map_names[$map_count]="$map_name"
                
                echo "   [$map_count] $map_name"
                if [ -f "$map_dir/info.txt" ]; then
                    echo "       $(head -n 1 $map_dir/info.txt)"
                fi
                
                if [ -f "$map_dir/delivery_zones.yaml" ]; then
                    zone_count=$(grep -c "^  - name:" "$map_dir/delivery_zones.yaml" 2>/dev/null || echo "0")
                    echo "       Zones: $zone_count"
                fi
            fi
        done
        
        echo ""
        echo "   [0] Use current/default map"
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo -n "   Select map [0-$map_count]: "
        read -r -t 30 map_choice || map_choice="0"
        echo ""
        
        if [ "$map_choice" -gt 0 ] && [ "$map_choice" -le "$map_count" ]; then
            selected_map="${map_names[$map_choice]}"
            echo "   Loading map: $selected_map"
            
            # Load the selected map
            "$SCRIPT_DIR/map_manager.sh" load "$selected_map"
            
            echo ""
        elif [ "$map_choice" = "0" ]; then
            echo "   Using current/default map"
            echo ""
        else
            echo "   Invalid selection, using current/default map"
            echo ""
        fi
    fi
fi

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built! Please run './scripts/build_project.sh' first."
    exit 1
fi

source install/setup.bash

# Run comprehensive cleanup to prevent TF errors
echo "🧹 Checking for stale processes..."
STALE_PROCESSES=$(pgrep -f "slam_toolbox\|autonomous_slam_node\|delivery_robot_node\|inspection_robot_node" | tr '\n' ' ')
if [ ! -z "$STALE_PROCESSES" ]; then
    echo "   Found stale processes: $STALE_PROCESSES"
    echo "   Running cleanup script..."
    "$SCRIPT_DIR/cleanup_slam_processes.sh"
else
    echo "   ✅ No stale processes found"
    echo ""
fi

# Check for and kill any existing rosbridge instances
EXISTING_ROSBRIDGE=$(pgrep -f "rosbridge_websocket" | tr '\n' ' ')
if [ ! -z "$EXISTING_ROSBRIDGE" ]; then
    echo "🔍 Found existing rosbridge instance(s): $EXISTING_ROSBRIDGE"
    echo "   Cleaning up old rosbridge processes..."
    
    for pid in $EXISTING_ROSBRIDGE; do
        if ps -p $pid > /dev/null 2>&1; then
            # Try graceful shutdown first
            kill -TERM $pid 2>/dev/null
        fi
    done
    
    # Wait a moment for graceful shutdown
    sleep 2
    
    # Force kill any that didn't respond
    for pid in $EXISTING_ROSBRIDGE; do
        if ps -p $pid > /dev/null 2>&1; then
            kill -9 $pid 2>/dev/null
        fi
    done
    
    echo "   ✅ Old rosbridge instances cleaned up"
    echo ""
fi

# Check if Gazebo is running
USE_PHYSICAL_ROBOT=false
if ! pgrep -f "gz sim" > /dev/null; then
    echo "⚠️  Gazebo is not running!"
    echo ""
    echo "Would you like to run on physical TurtleBot3 instead? (y/n)"
    read -r response
    
    if [[ "$response" =~ ^[Yy]$ ]]; then
        USE_PHYSICAL_ROBOT=true
        echo ""
        echo "🤖 Switching to Physical TurtleBot3 Mode"
        echo "========================================"
        echo ""
        echo "Prerequisites:"
        echo "1. TurtleBot3 must be powered on and connected"
        echo "2. Hardware bringup must be running on TurtleBot"
        echo ""
        echo "💡 To start hardware bringup, run:"
        echo "   ./scripts/turtlebot_bringup.sh start"
        echo ""
        echo "Press Enter to continue or Ctrl+C to cancel..."
        read -r
    else
        echo ""
        echo "❌ Please start Gazebo first with:"
        echo "   ./launch_mgen.sh"
        exit 1
    fi
else
    echo "✅ Gazebo is running - using simulation mode"
    echo ""
fi

# Function to offer mode selection after exploration completes
offer_mode_selection() {
    # Alert the user (beep if available)
    echo -e "\a"  # Terminal bell
    sleep 0.5
    echo -e "\a"
    sleep 0.5
    echo -e "\a"
    
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "🎉 EXPLORATION PHASE COMPLETE!"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "   ✓ Warehouse fully mapped"
    echo "   ✓ Robot at home position (0, 0)"
    echo "   ✓ Map saved and ready"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "🤖 SELECT ROBOT MODE"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "   Choose which robot mode to activate:"
    echo ""
    echo "   [1] 🗺️  EXPLORATION MODE"
    echo "       • Autonomous frontier exploration"
    echo "       • Create new map from scratch"
    echo "       • SLAM mapping mode"
    echo "       • Replaces existing map"
    echo ""
    echo "   [2] 📍 DEFINE DELIVERY ZONES"
    echo "       • Click points in RViz to mark zones"
    echo "       • Visualize zones on map"
    echo "       • Save zones for delivery mode"
    echo "       • Edit existing zones"
    echo ""
    echo "   [3] 📦 DELIVERY MODE"
    echo "       • Multi-point delivery operations"
    echo "       • Uses saved delivery zones"
    echo "       • Route optimization (TSP)"
    echo "       • Delivery logging to CSV"
    echo ""
    echo "   [4] 💾 SAVE CURRENT MAP"
    echo "       • Save map with custom name"
    echo "       • Includes zones and robot pose"
    echo "       • Can be loaded later"
    echo ""
    echo "   [5] 🔍 INSPECTION EXPLORATION MODE"
    echo "       • Autonomous exploration with AprilTag detection"
    echo "       • Discovers and marks damage sites automatically"
    echo "       • Saves damage site locations to file"
    echo "       • Similar to frontier exploration but with camera"
    echo ""
    echo "   [6] 📋 INSPECTION MODE"
    echo "       • Navigate to pre-defined damage sites"
    echo "       • Read AprilTag IDs with camera"
    echo "       • Log inspection results"
    echo "       • Route optimization (TSP)"
    echo ""
    echo "   [7] 💾 SAVE CURRENT MAP"
    echo "       • Save map with custom name"
    echo "       • Includes zones and robot pose"
    echo "       • Can be loaded later"
    echo ""
    echo "   [8] ❌ EXIT"
    echo "       • Shutdown system"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "   Enter your choice [1/2/3/4/5/6/7/8]"
    echo "   (You have 60 seconds to respond)"
    echo ""
    echo -n "   👉 Your choice: "
    read -r -t 60 response || response="8"
    echo ""
    
    if [[ "$response" == "1" ]]; then
        echo ""
        echo "🗺️  Exploration Mode - Creating New Map"
        echo "========================================"
        echo ""
        echo "⚠️  WARNING: This will replace your existing map!"
        echo ""
        echo -n "Are you sure you want to start exploration? (yes/no): "
        read -r confirm
        
        if [[ "$confirm" != "yes" ]]; then
            echo "Exploration cancelled. Returning to menu..."
            sleep 1
            offer_mode_selection
            return
        fi
        
        echo ""
        echo "🔄 Switching to exploration mode..."
        echo ""
        
        # Check if SLAM Toolbox is already in mapping mode
        SLAM_IN_MAPPING_MODE=false
        if [ "$PRELOAD_MAP" = false ] && [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
            # SLAM Toolbox is already running in mapping mode (started at script launch)
            SLAM_IN_MAPPING_MODE=true
            echo "   ✓ SLAM Toolbox already running in mapping mode"
        fi
        
        # Only restart SLAM if we need to switch from localization to mapping
        if [ "$SLAM_IN_MAPPING_MODE" = false ]; then
            echo "   ⚠️  Need to switch SLAM Toolbox from localization to mapping mode"
            echo "   This may cause TF transform issues (white robot)"
            echo ""
            echo "   RECOMMENDED: Restart the entire system for clean TF tree"
            echo "   1. Press Ctrl+C to exit this script"
            echo "   2. Stop Gazebo (Ctrl+C in Gazebo terminal)"
            echo "   3. Restart: ./launch_warehouse.sh"
            echo "   4. Run this script WITHOUT -preload flag"
            echo ""
            echo -n "   Continue anyway? (yes/no): "
            read -r tf_confirm
            
            if [[ "$tf_confirm" != "yes" ]]; then
                echo "   Returning to menu..."
                sleep 1
                offer_mode_selection
                return
            fi
            
            echo ""
            echo "   Attempting mode switch (may have TF issues)..."
            echo ""
            
            # Stop SLAM Toolbox (localization mode)
            if [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                echo "   Stopping SLAM Toolbox (localization)..."
                kill -TERM $SLAM_TOOLBOX_PID 2>/dev/null
                sleep 2
            fi
            
            # Give extra time for TF to clear
            echo "   Waiting for TF transforms to clear..."
            sleep 3
            
            # Start SLAM Toolbox in mapping mode
            echo "   Starting SLAM Toolbox in MAPPING mode..."
            ros2 launch slam_toolbox online_async_launch.py use_sim_time:=$USE_SIM_TIME > /tmp/slam_toolbox.log 2>&1 &
            SLAM_TOOLBOX_PID=$!
            sleep 5
            
            if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                echo "   ❌ Failed to start SLAM Toolbox in mapping mode"
                offer_mode_selection
                return
            fi
            
            echo "   ✓ SLAM Toolbox started in mapping mode"
        fi
        
        # Start exploration node
        echo "   Starting Autonomous SLAM Controller..."
        "$SCRIPT_DIR/start_autonomous_slam_clean.sh" "$(pwd)" &
        SLAM_PID=$!
        sleep 2
        
        if ! ps -p $SLAM_PID > /dev/null 2>&1; then
            echo "   ❌ Failed to start exploration controller"
            offer_mode_selection
            return
        fi
        
        echo ""
        echo "✅ Exploration mode active!"
        echo "   Robot will now explore and create a new map"
        echo "   Press Ctrl+C to stop"
        echo ""
        
        # Wait for exploration to complete
        while ps -p $SLAM_PID > /dev/null 2>&1; do
            sleep 1
        done
        
        echo ""
        echo "✅ Exploration complete!"
        sleep 2
        offer_mode_selection
        
    elif [[ "$response" == "2" ]]; then
        echo ""
        echo "📍 Zone Definition Mode"
        echo "======================="
        echo ""
        echo "   Starting zone marker visualization..."
        echo ""
        
        # Start zone marker node
        ros2 run warehouse_robot_system zone_marker_node &
        ZONE_MARKER_PID=$!
        sleep 2
        
        echo "✅ Zone Definition Mode Active!"
        echo ""
        echo "📋 Instructions:"
        echo "   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "   1️⃣  Open RViz (should already be open)"
        echo "   2️⃣  Add MarkerArray display if not visible:"
        echo "       • Click 'Add' button in RViz"
        echo "       • Select 'MarkerArray'"
        echo "       • Set Topic to: /delivery_zones/markers"
        echo "   3️⃣  Use 'Publish Point' tool (top toolbar)"
        echo "   4️⃣  Click on the map to add delivery zones"
        echo "   5️⃣  Each click creates a new zone (Zone_1, Zone_2, etc.)"
        echo "   6️⃣  Zones appear as colored cylinders on the map"
        echo ""
        echo "💡 Tips:"
        echo "   • Click on accessible (white) areas only"
        echo "   • Avoid obstacles (black areas)"
        echo "   • Zones are automatically saved"
        echo "   • You can run this mode again to add more zones"
        echo ""
        echo "📁 Zones saved to: $(pwd)/delivery_zones.yaml"
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "Commands:"
        echo "  • Press ENTER to finish and return to menu"
        echo "  • Type 'clear' to delete all zones and start over"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo -n "👉 "
        read -r user_input
        
        # Check if user wants to clear zones
        if [[ "$user_input" == "clear" ]]; then
            echo ""
            echo "🗑️  Clearing all zones..."
            
            # Call service to clear zones
            ros2 service call /clear_zones std_srvs/srv/Trigger
            
            echo "✅ All zones cleared! You can now add new zones."
            echo ""
            echo "Press ENTER when done to return to menu..."
            read -r
        fi
        
        # Cleanup zone marker node
        if [ ! -z "$ZONE_MARKER_PID" ] && ps -p $ZONE_MARKER_PID > /dev/null 2>&1; then
            echo ""
            echo "💾 Saving zones and stopping marker node..."
            kill -TERM $ZONE_MARKER_PID 2>/dev/null
            sleep 1
        fi
        
        echo ""
        echo "✅ Zones saved! Returning to mode selection..."
        sleep 1
        offer_mode_selection
        
    elif [[ "$response" == "3" ]]; then
        echo ""
        echo "🔄 Switching to Delivery Mode..."
        echo "================================"
        echo ""
        
        # Prompt for route optimization mode
        echo "📊 SELECT ROUTE OPTIMIZATION MODE"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "   [1] 📋 ORDERED MODE (Sequential)"
        echo "       • Visits zones in defined order"
        echo "       • Zone_1 → Zone_2 → Zone_3 → ... → Home"
        echo "       • Fast startup, predictable route"
        echo "       • Good for pre-planned sequences"
        echo ""
        echo "   [2] 🎯 OPTIMIZED MODE (TSP)"
        echo "       • Finds shortest total path"
        echo "       • Uses A* distance matrix"
        echo "       • Simulated Annealing optimization"
        echo "       • Minimizes total travel distance"
        echo "       • Best for efficiency"
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo -n "   👉 Your choice [1/2]: "
        read -r opt_mode
        echo ""
        
        # Set environment variable based on choice
        if [[ "$opt_mode" == "2" ]]; then
            export DELIVERY_OPTIMIZATION="tsp"
            echo "   ✅ Selected: OPTIMIZED MODE (TSP)"
            echo "   Using A* + Simulated Annealing for route optimization"
        else
            export DELIVERY_OPTIMIZATION="ordered"
            echo "   ✅ Selected: ORDERED MODE (Sequential)"
            echo "   Zones will be visited in defined order"
        fi
        echo ""
        
        # Find the most recent map file
        MAP_FILE_BASE="$(pwd)/warehouse_map_final"
        if [ ! -f "${MAP_FILE_BASE}.yaml" ]; then
            MAP_FILE_BASE="$(pwd)/warehouse_map_complete"
        fi
        
        # Check if we're already in preload mode (SLAM already in localization)
        if [ "$PRELOAD_MAP" = true ]; then
            echo "   ✓ Already in localization mode (preload)"
            echo "   ✓ SLAM Toolbox running with loaded map"
            echo "   ✓ Skipping SLAM restart to preserve TF tree"
            echo ""
        elif [ ! -f "${MAP_FILE_BASE}.yaml" ]; then
            echo "   ⚠️  No saved map found, continuing with current SLAM state"
            echo "   SLAM Toolbox will continue in mapping mode"
        else
            # Stop SLAM Toolbox (mapping mode) and restart in localization mode
            if [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                echo "   Switching SLAM Toolbox: mapping → localization mode..."
                kill -TERM $SLAM_TOOLBOX_PID 2>/dev/null
                sleep 2
                
                # Wait for clean shutdown
                for i in {1..5}; do
                    if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                        break
                    fi
                    sleep 0.5
                done
            fi
            
            # Read the saved robot pose if available
            POSE_FILE="${MAP_FILE_BASE}_pose.txt"
            if [ -f "$POSE_FILE" ]; then
                echo "   Found saved robot pose: $POSE_FILE"
                # The pose file will be used to set initial pose via ROS 2 service after launch
            fi
            
            # Create temporary params file with the correct map path
            TEMP_PARAMS="/tmp/slam_localization_params_$$.yaml"
            cat > "$TEMP_PARAMS" << EOF
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: false
    mode: localization
    map_file_name: $MAP_FILE_BASE
    map_start_at_dock: true
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 3.5
    minimum_time_interval: 0.5
    transform_publish_period: 0.02
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    correlation_search_space_dimension: 0.3
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.05
    distance_variance_penalty: 0.3
    angle_variance_penalty: 0.5
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
EOF
            
            echo "   Loading map: ${MAP_FILE_BASE}.yaml"
            
            # Start SLAM Toolbox in localization mode
            ros2 launch slam_toolbox localization_launch.py \
                use_sim_time:=$USE_SIM_TIME \
                slam_params_file:=$TEMP_PARAMS > /tmp/slam_toolbox.log 2>&1 &
            SLAM_TOOLBOX_PID=$!
            
            echo "   Waiting for SLAM Toolbox to initialize..."
            sleep 5
            
            if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                echo "   ❌ Failed to start SLAM in localization mode"
                echo "   Check logs: tail -f /tmp/slam_toolbox.log"
                return 1
            fi
            
            echo "   ✅ SLAM Toolbox running in localization mode"
            
            # Set initial pose if pose file exists
            if [ -f "$POSE_FILE" ]; then
                echo "   Setting initial robot pose from saved data..."
                sleep 2  # Give SLAM Toolbox time to fully initialize
                
                # Read pose from file (format: x y z roll pitch yaw_qz yaw_qw)
                read X Y Z ROLL PITCH QZ QW < "$POSE_FILE"
                
                echo "   Initial pose: x=$X, y=$Y, qz=$QZ, qw=$QW"
                
                # Publish initial pose to /initialpose topic
                ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
                  header: {
                    stamp: {sec: 0, nanosec: 0},
                    frame_id: 'map'
                  },
                  pose: {
                    pose: {
                      position: {x: $X, y: $Y, z: 0.0},
                      orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}
                    },
                    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]
                  }
                }" > /dev/null 2>&1 &
                
                echo "   ✅ Initial pose set"
            fi
        fi
        
        # Wait for SLAM to be ready (check if /map topic is publishing)
        echo "   Waiting for SLAM to be ready..."
        WAIT_COUNT=0
        MAX_WAIT=30
        MAP_READY=false
        
        while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
            if ros2 topic info /map > /dev/null 2>&1; then
                # Topic exists, now check if it's publishing
                if timeout 2s ros2 topic echo /map --once > /dev/null 2>&1; then
                    MAP_READY=true
                    echo "   ✅ Map is being published"
                    break
                fi
            fi
            sleep 1
            WAIT_COUNT=$((WAIT_COUNT + 1))
            if [ $((WAIT_COUNT % 5)) -eq 0 ]; then
                echo "   Still waiting for map... ($WAIT_COUNT/${MAX_WAIT}s)"
            fi
        done
        
        if [ "$MAP_READY" = false ]; then
            echo "   ⚠️  WARNING: Map not ready after ${MAX_WAIT}s"
            echo "   The delivery robot may not work correctly without a map!"
            echo "   Check SLAM Toolbox logs: tail -f /tmp/slam_toolbox.log"
            sleep 2
        fi
        
        # Start delivery robot node
        echo "   Starting Delivery Robot node..."
        ros2 run warehouse_robot_system delivery_robot_node --ros-args -p battery_low_threshold:=$BATTERY_THRESHOLD &
        DELIVERY_PID=$!
        sleep 3
        
        if ! ps -p $DELIVERY_PID > /dev/null 2>&1; then
            echo "   ❌ Failed to start delivery robot"
            return 1
        fi
        
        echo "   ✅ Delivery Robot node started"
        echo ""
        echo "✅ =============================================="
        echo "   DELIVERY MODE ACTIVE!"
        echo "   =============================================="
        echo ""
        echo "📋 Delivery Robot Workflow:"
        echo "   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "   STEP 1: Define Delivery Zones"
        echo "   • Use RViz 'Publish Point' tool"
        echo "   • Click on map to add zones"
        echo "   • Each click = new zone (Zone_1, Zone_2, etc.)"
        echo ""
        echo "   STEP 2: Save Zones (in another terminal)"
        echo "   $ ./scripts/delivery_commands.sh save"
        echo ""
        echo "   STEP 3: Start Deliveries"
        echo "   $ ./scripts/delivery_commands.sh start"
        echo ""
        echo "   STEP 4: Monitor Progress"
        echo "   $ ./scripts/delivery_commands.sh status"
        echo ""
        echo "📁 Output Files:"
        echo "   • delivery_zones.yaml - Saved zones"
        echo "   • delivery_log.csv - Delivery records"
        echo ""
        echo "💡 Quick Commands:"
        echo "   ./scripts/delivery_commands.sh save    # Save zones"
        echo "   ./scripts/delivery_commands.sh start   # Begin deliveries"
        echo "   ./scripts/delivery_commands.sh status  # Watch progress"
        echo "   ./scripts/delivery_commands.sh log     # View history"
        echo ""
        echo "Press Ctrl+C to stop delivery system"
        echo ""
        
        # Set up new trap for delivery mode
        trap 'cleanup_delivery_mode' SIGINT SIGTERM
        
        # Wait in delivery mode
        DELIVERY_COMPLETE=false
        while true; do
            # Check for delivery completion marker
            if [ -f "/tmp/delivery_complete.marker" ]; then
                echo ""
                echo "✅ Delivery system completed all tasks!"
                rm -f "/tmp/delivery_complete.marker"
                DELIVERY_COMPLETE=true
                break
            fi
            
            if ! ps -p $DELIVERY_PID > /dev/null 2>&1; then
                echo ""
                echo "ℹ️  Delivery Robot process exited"
                # Check if it was a clean completion
                if [ -f "/tmp/delivery_complete.marker" ]; then
                    DELIVERY_COMPLETE=true
                    rm -f "/tmp/delivery_complete.marker"
                fi
                break
            fi
            
            if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                echo "❌ SLAM Toolbox process died!"
                break
            fi
            
            if [ "$USE_PHYSICAL_ROBOT" = false ] && [ ! -z "$RSP_PID" ]; then
                if ! ps -p $RSP_PID > /dev/null 2>&1; then
                    echo "❌ robot_state_publisher process died!"
                    break
                fi
            fi
            
            sleep 1
        done
        
        # Cleanup delivery mode
        if [ ! -z "$DELIVERY_PID" ] && ps -p $DELIVERY_PID > /dev/null 2>&1; then
            echo "   Stopping Delivery Robot..."
            kill -TERM $DELIVERY_PID 2>/dev/null
            sleep 0.5
        fi
        
        # If deliveries completed successfully, offer mode selection again
        if [ "$DELIVERY_COMPLETE" = true ]; then
            echo ""
            echo "   Deliveries completed successfully!"
            echo "   Returning to mode selection..."
            sleep 2
            offer_mode_selection
        fi
        
    elif [[ "$response" == "4" ]]; then
        echo ""
        echo "💾 Save Current Map"
        echo "==================="
        echo ""
        echo -n "   Enter map name: "
        read -r map_name
        
        if [ -z "$map_name" ]; then
            echo "   ❌ Map name cannot be empty"
            sleep 2
            offer_mode_selection
            return
        fi
        
        echo -n "   Enter description (optional): "
        read -r map_desc
        
        echo ""
        "$SCRIPT_DIR/map_manager.sh" save "$map_name" "$map_desc"
        
        echo "   Press ENTER to return to menu..."
        read -r
        offer_mode_selection
        
    elif [[ "$response" == "5" ]]; then
        echo ""
        echo "🔍 Inspection Exploration Mode - Discovering Damage Sites"
        echo "=========================================================="
        echo ""
        echo "This mode will:"
        echo "  • Systematically patrol the already-mapped warehouse"
        echo "  • Detect AprilTags and colors with the camera"
        echo "  • Automatically mark damage site locations"
        echo "  • Save discovered sites to damage_sites.yaml"
        echo ""
        echo "⚠️  REQUIREMENTS:"
        echo "  • Existing map (run exploration mode first!)"
        echo "  • Camera working (/camera/image_raw)"
        echo "  • SLAM Toolbox in localization mode"
        echo ""
        echo "📋 How it works:"
        echo "  1. Robot patrols all accessible areas systematically"
        echo "  2. AprilTag & Color detectors run continuously"
        echo "  3. When damage detected, location is auto-saved"
        echo "  4. Returns home when patrol complete"
        echo ""
        echo "Would you like to:"
        echo "  [1] Start Inspection Exploration (systematic patrol)"
        echo "  [2] Return to menu"
        echo ""
        echo -n "👉 Your choice: "
        read -r explore_choice
        
        if [[ "$explore_choice" == "1" ]]; then
            echo ""
            echo "🔍 Starting Inspection Exploration with AprilTag Detection..."
            echo "============================================================"
            echo ""
            
            # Check if we have a valid map
            if ! ros2 topic info /map > /dev/null 2>&1; then
                echo "❌ No map available! Please run exploration mode first."
                echo "   The inspection exploration needs an existing map to patrol."
                sleep 3
                offer_mode_selection
                return
            fi
            
            echo "✅ Map detected - ready for inspection exploration"
            echo ""
            
            # Check camera availability and stability
            echo "📹 Checking camera feed..."
            CAMERA_CHECK_TIMEOUT=10
            CAMERA_READY=false
            
            for i in $(seq 1 $CAMERA_CHECK_TIMEOUT); do
                # Check if topic exists and has at least one publisher
                if ros2 topic info /camera/image_raw 2>/dev/null | grep -q "Publisher count: [1-9]"; then
                    # Try to get one message to verify it's actually publishing
                    if timeout 3s ros2 topic echo /camera/image_raw --once > /dev/null 2>&1; then
                        CAMERA_READY=true
                        echo "   ✅ Camera feed detected and stable"
                        break
                    fi
                fi
                echo "   Waiting for camera... ($i/$CAMERA_CHECK_TIMEOUT)"
                sleep 1
            done
            
            if [ "$CAMERA_READY" = false ]; then
                echo ""
                echo "   ❌ Camera feed not available or unstable!"
                echo "   Camera topic /camera/image_raw is not publishing"
                echo ""
                echo "   Please check:"
                echo "   • Is the camera node running?"
                echo "   • Is the TurtleBot connected?"
                echo "   • Network connection stable?"
                echo ""
                echo -n "   Continue anyway? (yes/no): "
                read -r camera_continue
                
                if [[ "$camera_continue" != "yes" ]]; then
                    echo "   Returning to menu..."
                    sleep 1
                    offer_mode_selection
                    return
                fi
                echo "   ⚠️  Proceeding without stable camera feed"
            fi
            
            # Determine visualization mode
            SHOW_CAMERA_UI=true
            if [ "$DISABLE_CAMERA_UI" = true ]; then
                SHOW_CAMERA_UI=false
                echo "   📷 Camera UI disabled (-nocamui flag)"
            fi
            
            # ═══════════════════════════════════════════════════════════════
            # CAMERA DETECTION NOW RUNS ON TURTLEBOT (NOT LAPTOP)
            # Start camera on TurtleBot with: ~/turtlebot_start_camera.sh
            # ═══════════════════════════════════════════════════════════
            echo "   📹 Camera detection running on TurtleBot"
            echo "   (AprilTag & Color detectors on TurtleBot)"
            
            # Check if AprilTag detections are being published from TurtleBot
            if timeout 3s ros2 topic info /apriltag_detections > /dev/null 2>&1; then
                echo "   ✅ AprilTag detections available from TurtleBot"
            else
                echo "   ⚠️  AprilTag detections not available"
                echo "   Make sure camera is running on TurtleBot:"
                echo "   ssh ubuntu@10.42.0.1 '~/turtlebot_start_camera.sh'"
            fi
            
            # OLD CODE (commented out - camera now on TurtleBot):
            # if ! pgrep -f "apriltag_detector_node" > /dev/null; then
            #     echo "   Starting AprilTag detector..."
            #     ros2 run warehouse_robot_system apriltag_detector_node ...
            # fi
            
            # OLD CODE (all commented out - camera now on TurtleBot)
            
            # ═══════════════════════════════════════════════════════════════
            # CAMERA NODE & COLOR DETECTOR NOW RUN ON TURTLEBOT
            # These are no longer needed on the laptop
            # ═══════════════════════════════════════════════════════════
            
            # OLD CODE (commented out - camera now on TurtleBot):
            # if ! pgrep -f "camera_node" > /dev/null; then
            #     ros2 run warehouse_robot_system camera_node ...
            # fi
            # if ! pgrep -f "colour_detector_node" > /dev/null; then
            #     ros2 run warehouse_robot_system colour_detector_node ...
            # fi
            
            # Wait for TF transforms to stabilize (reduced since camera is on TurtleBot now)
            echo ""
            echo "🔄 Waiting for TF transforms to stabilize..."
            echo ""
            
            TF_STABLE=false
            TF_CHECK_COUNT=0
            MAX_TF_CHECKS=3  # Reduced from 15 - camera on TurtleBot means less network load
            TOTAL_ATTEMPTS=0
            
            while [ $TOTAL_ATTEMPTS -lt $MAX_TF_CHECKS ]; do
                TOTAL_ATTEMPTS=$((TOTAL_ATTEMPTS + 1))
                
                # Check if critical transforms are available and publishing (with timeout)
                if timeout 2s bash -c "ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q 'At time'" 2>/dev/null; then
                    # Transform exists and is publishing, check if it's stable
                    TF_CHECK_COUNT=$((TF_CHECK_COUNT + 1))
                    
                    if [ $TF_CHECK_COUNT -ge 2 ]; then
                        # Got 2 consecutive successful checks
                        TF_STABLE=true
                        echo "   ✅ TF transforms stable (map → base_footprint)"
                        break
                    fi
                    echo "   Checking TF stability... ($TF_CHECK_COUNT/2)"
                else
                    # Transform failed, reset counter
                    TF_CHECK_COUNT=0
                    if [ $((TOTAL_ATTEMPTS % 3)) -eq 0 ]; then
                        echo "   Waiting for TF transforms... ($TOTAL_ATTEMPTS/$MAX_TF_CHECKS)"
                    fi
                fi
                
                sleep 0.5
            done
            
            if [ "$TF_STABLE" = false ]; then
                echo "   ℹ️  TF transforms still initializing (this is normal with camera on TurtleBot)"
            fi
            
            # Brief wait for robot to appear in RViz
            sleep 1
            
            # Start inspection robot in exploration mode
            echo "   Starting Inspection Robot in exploration mode..."
            echo ""
            export INSPECTION_MODE="exploration"
            ros2 run warehouse_robot_system inspection_robot_node --ros-args -p battery_low_threshold:=$BATTERY_THRESHOLD &
            INSPECTION_PID=$!
            sleep 3
            
            if ! ps -p $INSPECTION_PID > /dev/null 2>&1; then
                echo "   ❌ Failed to start inspection robot"
                echo "   Check log: /tmp/inspection_exploration.log"
                offer_mode_selection
                return
            fi
            
            # Final check: Is robot visible in RViz?
            echo ""
            echo "   🤖 Robot Status Check"
            echo "   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            echo "   Please check RViz:"
            echo "   • Is the robot visible (not white)?"
            echo "   • Is the robot at the correct position?"
            echo "   • Are the laser scans showing?"
            echo ""
            echo -n "   Is the robot visible and ready? (yes/no): "
            read -r -t 20 robot_visible || robot_visible="yes"
            echo ""
            
            if [[ "$robot_visible" != "yes" ]]; then
                echo "   ⚠️  Robot not visible in RViz"
                echo ""
                echo "   This usually means TF transforms are broken."
                echo "   The robot will not be able to navigate properly."
                echo ""
                echo "   RECOMMENDED FIX:"
                echo "   1. Press Ctrl+C to stop"
                echo "   2. Stop Gazebo/TurtleBot"
                echo "   3. Restart everything fresh"
                echo "   4. Use: ./scripts/run_autonomous_slam.sh -preload"
                echo ""
                echo -n "   Stop now and fix? (yes/no): "
                read -r -t 15 stop_now || stop_now="no"
                
                if [[ "$stop_now" == "yes" ]]; then
                    echo "   Stopping inspection robot..."
                    kill -TERM $INSPECTION_PID 2>/dev/null
                    sleep 1
                    offer_mode_selection
                    return
                fi
                
                echo "   ⚠️  Continuing with white robot (may not work correctly)"
            else
                echo "   ✅ Robot confirmed visible and ready!"
            fi
            
            echo ""
            echo "✅ ================================================"
            echo "   INSPECTION EXPLORATION ACTIVE!"
            echo "   ================================================"
            echo ""
            echo "🤖 Robot Behavior:"
            echo "   • Systematically patrols all accessible areas"
            echo "   • Uses grid-based coverage pattern"
            echo "   • Detects AprilTags and colors automatically"
            echo "   • Marks damage sites when tags detected"
            echo "   • Returns home when patrol complete"
            echo ""
            echo "📊 Monitor Progress:"
            echo "   • AprilTag detections: ros2 topic echo /apriltag_detections"
            echo "   • Color detections: ros2 topic echo /warehouse/damage_reports"
            echo "   • Robot status: ros2 topic echo /inspection/status"
            echo ""
            echo "📁 Output Files:"
            echo "   • damage_sites.yaml - Discovered damage locations"
            echo "   • inspection_exploration_log.csv - Detection log"
            echo ""
            echo "Press Ctrl+C to stop"
            echo ""
            
            # Set up trap for inspection exploration cleanup
            trap 'cleanup_inspection_exploration' SIGINT SIGTERM
            
            # Clean up any old marker files
            rm -f /tmp/inspection_exploration_complete.marker
            
            # Wait for inspection exploration to complete
            EXPLORATION_COMPLETE=false
            while true; do
                # Check for completion marker
                if [ -f "/tmp/inspection_exploration_complete.marker" ]; then
                    echo ""
                    echo "✅ Inspection exploration completed!"
                    rm -f "/tmp/inspection_exploration_complete.marker"
                    EXPLORATION_COMPLETE=true
                    break
                fi
                
                if ! ps -p $INSPECTION_PID > /dev/null 2>&1; then
                    echo ""
                    echo "ℹ️  Inspection Robot process exited"
                    if [ -f "/tmp/inspection_exploration_complete.marker" ]; then
                        EXPLORATION_COMPLETE=true
                        rm -f "/tmp/inspection_exploration_complete.marker"
                    fi
                    break
                fi
                
                sleep 1
            done
            
            # Cleanup
            if [ ! -z "$INSPECTION_PID" ] && ps -p $INSPECTION_PID > /dev/null 2>&1; then
                echo "   Stopping Inspection Robot..."
                kill -TERM $INSPECTION_PID 2>/dev/null
                sleep 0.5
            fi
            
            if [ ! -z "$APRILTAG_PID" ] && ps -p $APRILTAG_PID > /dev/null 2>&1; then
                echo "   Stopping AprilTag detector..."
                kill -TERM $APRILTAG_PID 2>/dev/null
                sleep 0.5
            fi
            
            if [ ! -z "$CAMERA_PID" ] && ps -p $CAMERA_PID > /dev/null 2>&1; then
                echo "   Stopping Camera node..."
                kill -TERM $CAMERA_PID 2>/dev/null
                sleep 0.5
            fi
            
            if [ ! -z "$COLOR_PID" ] && ps -p $COLOR_PID > /dev/null 2>&1; then
                echo "   Stopping Color detector..."
                kill -TERM $COLOR_PID 2>/dev/null
                sleep 0.5
            fi
            
            echo ""
            if [ "$EXPLORATION_COMPLETE" = true ]; then
                echo "✅ Inspection exploration completed successfully!"
                echo ""
                echo "📋 Results:"
                if [ -f "damage_sites.yaml" ]; then
                    SITE_COUNT=$(grep -c "^  - name:" "damage_sites.yaml" 2>/dev/null || echo "0")
                    echo "   • Discovered $SITE_COUNT damage sites"
                    echo "   • Sites saved to: damage_sites.yaml"
                else
                    echo "   • No damage sites discovered"
                fi
                
                if [ -f "inspection_exploration_log.csv" ]; then
                    echo "   • Detection log: inspection_exploration_log.csv"
                fi
                
                echo ""
                echo "💡 Next Steps:"
                echo "   1. Review discovered sites: ./scripts/inspection_commands.sh sites"
                echo "   2. Run inspection mode to verify sites: Select option [6]"
                echo "   3. Save map with sites: Select option [7]"
            else
                echo "⚠️  Inspection exploration stopped early"
                echo "   Check log: /tmp/inspection_exploration.log"
            fi
            
            echo ""
            sleep 3
            offer_mode_selection
        else
            echo "Returning to menu..."
            sleep 1
            offer_mode_selection
        fi
        return
        
    elif [[ "$response" == "6" ]]; then
        echo ""
        echo "🔄 Switching to Inspection Mode..."
        echo "=================================="
        echo ""
        
        # Prompt for route optimization mode
        echo "📊 SELECT ROUTE OPTIMIZATION MODE"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "   [1] 📋 ORDERED MODE (Sequential)"
        echo "       • Visits sites in defined order"
        echo "       • Damage_1 → Damage_2 → Damage_3 → ... → Home"
        echo "       • Fast startup, predictable route"
        echo ""
        echo "   [2] 🎯 OPTIMIZED MODE (TSP)"
        echo "       • Finds shortest total path"
        echo "       • Uses A* distance matrix"
        echo "       • Simulated Annealing optimization"
        echo "       • Minimizes total travel distance"
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo -n "   👉 Your choice [1/2]: "
        read -r opt_mode
        echo ""
        
        # Set environment variable based on choice
        if [[ "$opt_mode" == "2" ]]; then
            export INSPECTION_OPTIMIZATION="tsp"
            echo "   ✅ Selected: OPTIMIZED MODE (TSP)"
            echo "   Using A* + Simulated Annealing for route optimization"
        else
            export INSPECTION_OPTIMIZATION="ordered"
            echo "   ✅ Selected: ORDERED MODE (Sequential)"
            echo "   Sites will be visited in defined order"
        fi
        echo ""
        
        # Find the most recent map file
        MAP_FILE_BASE="$(pwd)/warehouse_map_final"
        if [ ! -f "${MAP_FILE_BASE}.yaml" ]; then
            MAP_FILE_BASE="$(pwd)/warehouse_map_complete"
        fi
        
        # Check if we're already in preload mode (SLAM already in localization)
        if [ "$PRELOAD_MAP" = true ]; then
            echo "   ✓ Already in localization mode (preload)"
            echo "   ✓ SLAM Toolbox running with loaded map"
            echo "   ✓ Skipping SLAM restart to preserve TF tree"
            echo ""
        elif [ ! -f "${MAP_FILE_BASE}.yaml" ]; then
            echo "   ⚠️  No saved map found, continuing with current SLAM state"
            echo "   SLAM Toolbox will continue in mapping mode"
        else
            # Stop SLAM Toolbox (mapping mode) and restart in localization mode
            if [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                echo "   Switching SLAM Toolbox: mapping → localization mode..."
                kill -TERM $SLAM_TOOLBOX_PID 2>/dev/null
                sleep 2
                
                for i in {1..5}; do
                    if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                        break
                    fi
                    sleep 0.5
                done
            fi
            
            # Create temporary params file
            TEMP_PARAMS="/tmp/slam_localization_params_$$.yaml"
            cat > "$TEMP_PARAMS" << EOF
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: false
    mode: localization
    map_file_name: $MAP_FILE_BASE
    map_start_at_dock: true
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 3.5
    minimum_time_interval: 0.5
    transform_publish_period: 0.02
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
EOF
            
            echo "   Loading map: ${MAP_FILE_BASE}.yaml"
            
            # Start SLAM Toolbox in localization mode
            ros2 launch slam_toolbox localization_launch.py \
                use_sim_time:=$USE_SIM_TIME \
                slam_params_file:=$TEMP_PARAMS > /tmp/slam_toolbox.log 2>&1 &
            SLAM_TOOLBOX_PID=$!
            
            echo "   Waiting for SLAM Toolbox to initialize..."
            sleep 5
            
            if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                echo "   ❌ Failed to start SLAM in localization mode"
                echo "   Check logs: tail -f /tmp/slam_toolbox.log"
                return 1
            fi
            
            echo "   ✅ SLAM Toolbox running in localization mode"
        fi
        
        # Wait for SLAM to be ready
        echo "   Waiting for SLAM to be ready..."
        WAIT_COUNT=0
        MAX_WAIT=30
        MAP_READY=false
        
        while [ $WAIT_COUNT -lt $MAX_WAIT ]; do
            if ros2 topic info /map > /dev/null 2>&1; then
                if timeout 2s ros2 topic echo /map --once > /dev/null 2>&1; then
                    MAP_READY=true
                    echo "   ✅ Map is being published"
                    break
                fi
            fi
            sleep 1
            WAIT_COUNT=$((WAIT_COUNT + 1))
            if [ $((WAIT_COUNT % 5)) -eq 0 ]; then
                echo "   Still waiting for map... ($WAIT_COUNT/${MAX_WAIT}s)"
            fi
        done
        
        if [ "$MAP_READY" = false ]; then
            echo "   ⚠️  WARNING: Map not ready after ${MAX_WAIT}s"
            echo "   The inspection robot may not work correctly without a map!"
            sleep 2
        fi
        
        # Check camera availability
        echo "   Checking camera feed..."
        CAMERA_CHECK_TIMEOUT=10
        CAMERA_READY=false
        
        for i in $(seq 1 $CAMERA_CHECK_TIMEOUT); do
            # Check if topic exists and has at least one publisher
            if ros2 topic info /camera/image_raw 2>/dev/null | grep -q "Publisher count: [1-9]"; then
                # Try to get one message to verify it's actually publishing
                if timeout 3s ros2 topic echo /camera/image_raw --once > /dev/null 2>&1; then
                    CAMERA_READY=true
                    echo "   ✅ Camera feed detected"
                    break
                fi
            fi
            if [ $i -eq 1 ]; then
                echo "   Waiting for camera..."
            fi
            sleep 1
        done
        
        if [ "$CAMERA_READY" = false ]; then
            echo "   ⚠️  Camera feed not available"
            echo "   Inspection will continue but AprilTag detection may not work"
        fi
        
        # Determine visualization mode
        SHOW_CAMERA_UI=true
        if [ "$DISABLE_CAMERA_UI" = true ]; then
            SHOW_CAMERA_UI=false
            echo "   📷 Camera UI disabled (-nocamui flag)"
        fi
        
        # ═══════════════════════════════════════════════════════════════
        # CAMERA DETECTION NOW RUNS ON TURTLEBOT (NOT LAPTOP)
        # Start camera on TurtleBot with: ~/turtlebot_start_camera.sh
        # ═══════════════════════════════════════════════════════════
        echo "   📹 Camera detection running on TurtleBot"
        
        # Check if AprilTag detections are available
        if timeout 3s ros2 topic info /apriltag_detections > /dev/null 2>&1; then
            echo "   ✅ AprilTag detections available from TurtleBot"
        else
            echo "   ⚠️  AprilTag detections not available"
            echo "   Make sure camera is running on TurtleBot:"
            echo "   ssh ubuntu@10.42.0.1 '~/turtlebot_start_camera.sh'"
        fi
        
        # OLD CODE (commented out - camera now on TurtleBot):
        # if ! pgrep -f "apriltag_detector_node" > /dev/null; then
        #     ros2 run warehouse_robot_system apriltag_detector_node ...
        # fi
        
        # Wait for TF transforms to stabilize (reduced since camera is on TurtleBot now)
        echo ""
        echo "   🔄 Waiting for TF transforms to stabilize..."
        
        TF_STABLE=false
        TF_CHECK_COUNT=0
        TOTAL_ATTEMPTS=0
        MAX_TF_CHECKS=3  # Reduced from 15 - camera on TurtleBot means less network load
        
        while [ $TOTAL_ATTEMPTS -lt $MAX_TF_CHECKS ]; do
            TOTAL_ATTEMPTS=$((TOTAL_ATTEMPTS + 1))
            
            # Check if transform is actually publishing data (with timeout)
            if timeout 2s bash -c "ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -q 'At time'" 2>/dev/null; then
                TF_CHECK_COUNT=$((TF_CHECK_COUNT + 1))
                if [ $TF_CHECK_COUNT -ge 2 ]; then
                    TF_STABLE=true
                    echo "   ✅ TF transforms stable"
                    break
                fi
                echo "   Checking TF stability... ($TF_CHECK_COUNT/2)"
            else
                TF_CHECK_COUNT=0
                if [ $((TOTAL_ATTEMPTS % 3)) -eq 0 ]; then
                    echo "   Waiting for TF transforms... ($TOTAL_ATTEMPTS/$MAX_TF_CHECKS)"
                fi
            fi
            sleep 0.5
        done
        
        if [ "$TF_STABLE" = false ]; then
            echo "   ⚠️  TF transforms not stable - robot may appear white in RViz"
        fi
        
        sleep 0.5  # Reduced from 2s - faster startup with camera on TurtleBot
        
        # Start inspection robot node
        echo "   Starting Inspection Robot node..."
        ros2 run warehouse_robot_system inspection_robot_node --ros-args -p battery_low_threshold:=$BATTERY_THRESHOLD &
        INSPECTION_PID=$!
        sleep 3
        
        if ! ps -p $INSPECTION_PID > /dev/null 2>&1; then
            echo "   ❌ Failed to start inspection robot"
            return 1
        fi
        
        echo "   ✅ Inspection Robot node started"
        
        # Check robot visibility
        echo ""
        echo "   🤖 Please verify robot is visible in RViz (not white)"
        echo -n "   Robot visible and ready? (yes/no): "
        read -r -t 15 robot_ok || robot_ok="yes"
        
        if [[ "$robot_ok" != "yes" ]]; then
            echo "   ⚠️  Robot visibility issue - consider restarting system"
        else
            echo "   ✅ Robot confirmed ready"
        fi
        
        echo ""
        echo "✅ =============================================="
        echo "   INSPECTION MODE ACTIVE!"
        echo "   =============================================="
        echo ""
        echo "📋 Inspection Robot Workflow:"
        echo "   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "   STEP 1: Define Damage Sites"
        echo "   • Use RViz 'Publish Point' tool"
        echo "   • Click on map to add damage sites"
        echo "   • Each click = new site (Damage_1, Damage_2, etc.)"
        echo ""
        echo "   STEP 2: Save Sites (in another terminal)"
        echo "   $ ./scripts/inspection_commands.sh save"
        echo ""
        echo "   STEP 3: Start Inspections"
        echo "   $ ./scripts/inspection_commands.sh start"
        echo ""
        echo "   STEP 4: Monitor Progress"
        echo "   $ ./scripts/inspection_commands.sh status"
        echo ""
        echo "📁 Output Files:"
        echo "   • damage_sites.yaml - Saved damage sites"
        echo "   • inspection_log.csv - Inspection records"
        echo ""
        echo "💡 Quick Commands:"
        echo "   ./scripts/inspection_commands.sh save    # Save sites"
        echo "   ./scripts/inspection_commands.sh start   # Begin inspections"
        echo "   ./scripts/inspection_commands.sh status  # Watch progress"
        echo "   ./scripts/inspection_commands.sh log     # View history"
        echo ""
        echo "🎥 Camera & AprilTag Detection:"
        echo "   • Robot reads AprilTag IDs at each site"
        echo "   • Detection timeout: 5 seconds per site"
        echo "   • Results logged to inspection_log.csv"
        echo ""
        echo "Press Ctrl+C to stop inspection system"
        echo ""
        
        # Set up new trap for inspection mode
        trap 'cleanup_inspection_mode' SIGINT SIGTERM
        
        # Wait in inspection mode
        INSPECTION_COMPLETE=false
        while true; do
            # Check for inspection completion marker
            if [ -f "/tmp/inspection_complete.marker" ]; then
                echo ""
                echo "✅ Inspection system completed all tasks!"
                rm -f "/tmp/inspection_complete.marker"
                INSPECTION_COMPLETE=true
                break
            fi
            
            if ! ps -p $INSPECTION_PID > /dev/null 2>&1; then
                echo ""
                echo "ℹ️  Inspection Robot process exited"
                if [ -f "/tmp/inspection_complete.marker" ]; then
                    INSPECTION_COMPLETE=true
                    rm -f "/tmp/inspection_complete.marker"
                fi
                break
            fi
            
            if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
                echo "❌ SLAM Toolbox process died!"
                break
            fi
            
            if [ "$USE_PHYSICAL_ROBOT" = false ] && [ ! -z "$RSP_PID" ]; then
                if ! ps -p $RSP_PID > /dev/null 2>&1; then
                    echo "❌ robot_state_publisher process died!"
                    break
                fi
            fi
            
            sleep 1
        done
        
        # Cleanup inspection mode
        if [ ! -z "$INSPECTION_PID" ] && ps -p $INSPECTION_PID > /dev/null 2>&1; then
            echo "   Stopping Inspection Robot..."
            kill -TERM $INSPECTION_PID 2>/dev/null
            sleep 0.5
        fi
        
        if [ ! -z "$APRILTAG_PID" ] && ps -p $APRILTAG_PID > /dev/null 2>&1; then
            echo "   Stopping AprilTag detector..."
            kill -TERM $APRILTAG_PID 2>/dev/null
            sleep 0.5
        fi
        
        # If inspections completed successfully, offer mode selection again
        if [ "$INSPECTION_COMPLETE" = true ]; then
            echo ""
            echo "   Inspections completed successfully!"
            echo "   Returning to mode selection..."
            sleep 2
            offer_mode_selection
        fi
        
    elif [[ "$response" == "7" ]]; then
        echo ""
        echo "💾 Save Current Map"
        echo "==================="
        echo ""
        echo -n "   Enter map name: "
        read -r map_name
        
        if [ -z "$map_name" ]; then
            echo "   ❌ Map name cannot be empty"
            sleep 2
            offer_mode_selection
            return
        fi
        
        echo -n "   Enter description (optional): "
        read -r map_desc
        
        echo ""
        "$SCRIPT_DIR/map_manager.sh" save "$map_name" "$map_desc"
        
        echo "   Press ENTER to return to menu..."
        read -r
        offer_mode_selection
        
    else
        echo ""
        echo "   Exiting - system shutdown complete"
    fi
}

# Cleanup function for delivery mode
cleanup_delivery_mode() {
    echo ""
    echo "🛑 Shutting down Delivery System..."
    
    if [ ! -z "$DELIVERY_PID" ] && ps -p $DELIVERY_PID > /dev/null 2>&1; then
        echo "   Stopping Delivery Robot..."
        kill -TERM $DELIVERY_PID 2>/dev/null
        sleep 0.5
    fi
    
    echo "✅ Delivery system stopped."
    exit 0
}

# Cleanup function for inspection mode
cleanup_inspection_mode() {
    echo ""
    echo "🛑 Shutting down Inspection System..."
    
    if [ ! -z "$INSPECTION_PID" ] && ps -p $INSPECTION_PID > /dev/null 2>&1; then
        echo "   Stopping Inspection Robot..."
        kill -TERM $INSPECTION_PID 2>/dev/null
        sleep 0.5
    fi
    
    if [ ! -z "$APRILTAG_PID" ] && ps -p $APRILTAG_PID > /dev/null 2>&1; then
        echo "   Stopping AprilTag detector..."
        kill -TERM $APRILTAG_PID 2>/dev/null
        sleep 0.5
    fi
    
    echo "✅ Inspection system stopped."
    exit 0
}

# Cleanup function for inspection exploration mode
cleanup_inspection_exploration() {
    echo ""
    echo "🛑 Shutting down Inspection Exploration System..."
    
    if [ ! -z "$INSPECTION_PID" ] && ps -p $INSPECTION_PID > /dev/null 2>&1; then
        echo "   Stopping Inspection Robot..."
        kill -TERM $INSPECTION_PID 2>/dev/null
        sleep 0.5
    fi
    
    if [ ! -z "$CAMERA_PID" ] && ps -p $CAMERA_PID > /dev/null 2>&1; then
        echo "   Stopping Camera node..."
        kill -TERM $CAMERA_PID 2>/dev/null
        sleep 0.5
    fi
    
    if [ ! -z "$APRILTAG_PID" ] && ps -p $APRILTAG_PID > /dev/null 2>&1; then
        echo "   Stopping AprilTag detector..."
        kill -TERM $APRILTAG_PID 2>/dev/null
        sleep 0.5
    fi
    
    if [ ! -z "$COLOR_PID" ] && ps -p $COLOR_PID > /dev/null 2>&1; then
        echo "   Stopping Color detector..."
        kill -TERM $COLOR_PID 2>/dev/null
        sleep 0.5
    fi
    
    echo "✅ Inspection exploration system stopped."
    exit 0
}

# Function to cleanup processes on exit
cleanup() {
    # Check if we should offer mode selection
    local OFFER_MODE_SELECTION=false
    
    # Check if exploration completed (look for completion marker in logs)
    if [ -f "/tmp/slam_exploration_complete.marker" ]; then
        OFFER_MODE_SELECTION=true
        rm -f "/tmp/slam_exploration_complete.marker"
    fi
    
    echo ""
    echo "🛑 Shutting down Autonomous SLAM System..."
    
    # Gracefully shutdown background processes
    if [ ! -z "$RSP_PID" ] && ps -p $RSP_PID > /dev/null 2>&1; then
        echo "   Stopping robot_state_publisher..."
        kill -TERM $RSP_PID 2>/dev/null
        sleep 0.5
    fi
    
    # Enhanced SLAM Toolbox cleanup - kill all instances
    if [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
        echo "   Stopping SLAM Toolbox (PID: $SLAM_TOOLBOX_PID)..."
        kill -TERM $SLAM_TOOLBOX_PID 2>/dev/null
        sleep 1
        # Force kill if still running
        if ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
            kill -9 $SLAM_TOOLBOX_PID 2>/dev/null
        fi
    fi
    
    # Kill any remaining SLAM Toolbox processes
    REMAINING_SLAM=$(pgrep -f "slam_toolbox" | tr '\n' ' ')
    if [ ! -z "$REMAINING_SLAM" ]; then
        echo "   Cleaning up remaining SLAM Toolbox processes: $REMAINING_SLAM"
        for pid in $REMAINING_SLAM; do
            kill -9 $pid 2>/dev/null
        done
        sleep 0.5
    fi
    
    if [ ! -z "$BATTERY_PID" ] && ps -p $BATTERY_PID > /dev/null 2>&1; then
        echo "   Stopping Battery Monitor..."
        kill -TERM $BATTERY_PID 2>/dev/null
        sleep 0.3
    fi
    
    if [ ! -z "$BATTERY_DISPLAY_PID" ] && ps -p $BATTERY_DISPLAY_PID > /dev/null 2>&1; then
        echo "   Stopping Battery Display..."
        kill -TERM $BATTERY_DISPLAY_PID 2>/dev/null
        sleep 0.3
    fi
    
    if [ ! -z "$ROSBRIDGE_PID" ] && ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
        echo "   Stopping rosbridge..."
        # Try graceful shutdown first (SIGTERM)
        kill -TERM $ROSBRIDGE_PID 2>/dev/null
        # Wait up to 3 seconds for graceful shutdown
        for i in {1..6}; do
            if ! ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
                break
            fi
            sleep 0.5
        done
        # Force kill if still running
        if ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
            kill -9 $ROSBRIDGE_PID 2>/dev/null
        fi
    fi
    
    if [ ! -z "$WEB_SERVER_PID" ] && ps -p $WEB_SERVER_PID > /dev/null 2>&1; then
        echo "   Stopping web server..."
        # Graceful shutdown for web server
        kill -TERM $WEB_SERVER_PID 2>/dev/null
        sleep 1
        if ps -p $WEB_SERVER_PID > /dev/null 2>&1; then
            kill -9 $WEB_SERVER_PID 2>/dev/null
        fi
    fi
    
    if [ ! -z "$RVIZ_PID" ] && ps -p $RVIZ_PID > /dev/null 2>&1; then
        echo "   Stopping RViz..."
        kill -TERM $RVIZ_PID 2>/dev/null
        sleep 0.5
    fi
    
    if [ ! -z "$SLAM_PID" ] && ps -p $SLAM_PID > /dev/null 2>&1; then
        echo "   Stopping Autonomous SLAM Controller..."
        kill -TERM $SLAM_PID 2>/dev/null
        sleep 0.5
    fi
    
    if [ ! -z "$SPAWN_PID" ] && ps -p $SPAWN_PID > /dev/null 2>&1; then
        echo "   Stopping TurtleBot3 spawn..."
        kill -TERM $SPAWN_PID 2>/dev/null
        sleep 0.3
    fi
    
    # Final cleanup - force kill any remaining processes
    echo "   Cleaning up any remaining processes..."
    for pid in $RSP_PID $SLAM_TOOLBOX_PID $BATTERY_PID $BATTERY_DISPLAY_PID $ROSBRIDGE_PID $WEB_SERVER_PID $RVIZ_PID $SLAM_PID $SPAWN_PID; do
        if [ ! -z "$pid" ] && ps -p $pid > /dev/null 2>&1; then
            kill -9 $pid 2>/dev/null
        fi
    done
    
    echo "✅ Autonomous SLAM system stopped."
    
    # If exploration was complete, offer mode selection
    if [ "$OFFER_MODE_SELECTION" = true ]; then
        offer_mode_selection
    fi
    
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo "🚀 Starting Autonomous SLAM Components..."
echo ""

# Different startup for physical robot vs simulation
if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "🤖 Physical TurtleBot3 Mode"
    echo "==========================="
    echo ""
    echo "Skipping robot spawn (using physical robot)"
    echo "Make sure hardware bringup is running on TurtleBot!"
    echo ""
    
    # Set use_sim_time to false for physical robot
    USE_SIM_TIME="False"
    RSP_PID=""
    SPAWN_PID=""
else
    echo "🎮 Simulation Mode"
    echo "=================="
    echo ""
    
    echo "1️⃣ Starting robot_state_publisher..."
    ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True &
    RSP_PID=$!
    sleep 2

    echo "2️⃣ Spawning TurtleBot3 at origin (0, 0) - autonomous mode..."
    ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0 &
    SPAWN_PID=$!
    sleep 4

    # Kill the wall-following drive node (we don't need it for autonomous SLAM)
    echo "🔧 Stopping wall-following node..."
    pkill -f turtlebot3_drive_node
    sleep 1

    echo "✅ Robot spawned without wall following - ready for autonomous SLAM control"
    
    # Set use_sim_time to true for simulation
    USE_SIM_TIME="True"
fi

if [ "$PRELOAD_MAP" = true ]; then
    echo "3️⃣ Starting SLAM Toolbox in LOCALIZATION mode (loading existing map)..."
    
    # Check if map exists (without .yaml extension for the parameter)
    MAP_FILE_BASE="$(pwd)/warehouse_map_final"
    if [ ! -f "${MAP_FILE_BASE}.yaml" ]; then
        MAP_FILE_BASE="$(pwd)/warehouse_map_complete"
    fi
    
    if [ ! -f "${MAP_FILE_BASE}.yaml" ]; then
        echo "❌ No map file found!"
        echo "   Looking for: warehouse_map_final.yaml or warehouse_map_complete.yaml"
        echo "   in: $(pwd)"
        echo "   Please run exploration first without -preload flag"
        cleanup
        exit 1
    fi
    
    echo "   Using map: ${MAP_FILE_BASE}.yaml"
    echo "   Map base path: $MAP_FILE_BASE"
    
    # Create temporary params file with the correct map path
    TEMP_PARAMS="/tmp/slam_localization_params_$$.yaml"
    cat > "$TEMP_PARAMS" << EOF
slam_toolbox:
  ros__parameters:
    # ROS Parameters
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan
    use_map_saver: false
    mode: localization
    
    # Map to load
    map_file_name: $MAP_FILE_BASE
    map_start_at_dock: true
    
    # SLAM Parameters
    map_update_interval: 5.0
    resolution: 0.05
    max_laser_range: 3.5
    minimum_time_interval: 0.5
    transform_publish_period: 0.02
    
    # Localization Parameters
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    
    # Scan Matcher Parameters
    correlation_search_space_dimension: 0.3
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.05
    
    distance_variance_penalty: 0.3
    angle_variance_penalty: 0.5
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
EOF
    
    echo "   Created temporary params file: $TEMP_PARAMS"
    
    # Launch with standard SLAM Toolbox localization launch
    ros2 launch slam_toolbox localization_launch.py \
        use_sim_time:=$USE_SIM_TIME \
        slam_params_file:=$TEMP_PARAMS > /tmp/slam_toolbox.log 2>&1 &
    SLAM_TOOLBOX_PID=$!
else
    echo "3️⃣ Starting SLAM Toolbox in MAPPING mode (output redirected to /tmp/slam_toolbox.log)..."
    
    # Use custom params for physical robot to handle time sync issues
    if [ "$USE_PHYSICAL_ROBOT" = true ]; then
        SLAM_PARAMS="$(pwd)/slam_toolbox_physical_robot.yaml"
        echo "   Using physical robot params with extended TF buffer: $SLAM_PARAMS"
        ros2 launch slam_toolbox online_async_launch.py \
            use_sim_time:=$USE_SIM_TIME \
            slam_params_file:=$SLAM_PARAMS > /tmp/slam_toolbox.log 2>&1 &
    else
        ros2 launch slam_toolbox online_async_launch.py use_sim_time:=$USE_SIM_TIME > /tmp/slam_toolbox.log 2>&1 &
    fi
    SLAM_TOOLBOX_PID=$!
fi

echo "⏳ Waiting for SLAM Toolbox to initialize..."
sleep 5

# Check if SLAM Toolbox started successfully
if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
    echo "❌ SLAM Toolbox failed to start!"
    echo "   Check logs: tail -f /tmp/slam_toolbox.log"
    cleanup
    exit 1
fi

# Wait for SLAM Toolbox to publish map frame
echo "   Waiting for SLAM Toolbox to publish TF frames..."
TF_WAIT_COUNT=0
TF_MAX_WAIT=15
MAP_FRAME_READY=false

while [ $TF_WAIT_COUNT -lt $TF_MAX_WAIT ]; do
    # Check if map frame exists in TF tree
    if ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "At time" 2>/dev/null; then
        MAP_FRAME_READY=true
        echo "   ✅ SLAM Toolbox TF frames ready (map → odom)"
        break
    fi
    sleep 1
    TF_WAIT_COUNT=$((TF_WAIT_COUNT + 1))
    if [ $((TF_WAIT_COUNT % 5)) -eq 0 ]; then
        echo "   Still waiting for TF frames... ($TF_WAIT_COUNT/${TF_MAX_WAIT}s)"
    fi
done

if [ "$MAP_FRAME_READY" = false ]; then
    echo "   ⚠️  WARNING: Map frame not ready after ${TF_MAX_WAIT}s"
    echo "   SLAM Toolbox may not be working correctly!"
    echo "   Check logs: tail -f /tmp/slam_toolbox.log"
    echo ""
    echo "   Continue anyway? (yes/no)"
    read -r -t 10 tf_response || tf_response="no"
    if [[ "$tf_response" != "yes" ]]; then
        cleanup
        exit 1
    fi
fi

echo "4️⃣ Starting rosbridge for web dashboard..."
# Check if rosbridge is available (suppress broken pipe errors)
if ros2 pkg list 2>/dev/null | grep -q "rosbridge_server" 2>/dev/null; then
    # Use clean launcher script to avoid conda library conflicts
    "$SCRIPT_DIR/start_rosbridge_clean.sh" "$(pwd)" > /tmp/rosbridge.log 2>&1 &
    ROSBRIDGE_PID=$!
    sleep 3
    
    if ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
        echo "   ✅ rosbridge started (PID: $ROSBRIDGE_PID)"
        
        # Start web server if -web flag was provided
        if [ "$START_WEB_DASHBOARD" = true ]; then
            echo "   Starting web server..."
            WEB_DIR="$(pwd)/src/mtrx3760_battery/web"
            
            # Check if node_modules exists
            if [ ! -d "$WEB_DIR/node_modules" ]; then
                echo "   📦 Installing web dependencies (first time only)..."
                cd "$WEB_DIR"
                npm install --silent > /tmp/npm_install.log 2>&1
                if [ $? -ne 0 ]; then
                    echo "   ⚠️  Failed to install web dependencies"
                    echo "   Check logs: tail -f /tmp/npm_install.log"
                    WEB_SERVER_PID=""
                    cd - > /dev/null
                else
                    cd - > /dev/null
                    # Start web server
                    cd "$WEB_DIR"
                    npm run dev > /tmp/battery_web.log 2>&1 &
                    WEB_SERVER_PID=$!
                    cd - > /dev/null
                    sleep 3
                    
                    if ps -p $WEB_SERVER_PID > /dev/null 2>&1; then
                        echo "   ✅ Web server started (PID: $WEB_SERVER_PID)"
                        echo "   🌐 Open browser: http://localhost:3000"
                    else
                        echo "   ⚠️  Web server failed to start"
                        WEB_SERVER_PID=""
                    fi
                fi
            else
                # Start web server
                cd "$WEB_DIR"
                npm run dev > /tmp/battery_web.log 2>&1 &
                WEB_SERVER_PID=$!
                cd - > /dev/null
                sleep 3
                
                if ps -p $WEB_SERVER_PID > /dev/null 2>&1; then
                    echo "   ✅ Web server started (PID: $WEB_SERVER_PID)"
                    echo "   🌐 Open browser: http://localhost:3000"
                else
                    echo "   ⚠️  Web server failed to start"
                    WEB_SERVER_PID=""
                fi
            fi
        else
            echo "   🌐 Web dashboard available at: http://localhost:3000"
            echo "   💡 To auto-start web server, use: $0 -web"
            WEB_SERVER_PID=""
        fi
    else
        echo "   ⚠️  rosbridge failed to start (check /tmp/rosbridge.log)"
        echo "   Web dashboard won't work without rosbridge"
        ROSBRIDGE_PID=""
        WEB_SERVER_PID=""
    fi
else
    echo "   ⚠️  rosbridge_server not installed"
    echo "   Install with: sudo apt install ros-jazzy-rosbridge-server"
    echo "   (Web dashboard will not be available)"
    ROSBRIDGE_PID=""
    WEB_SERVER_PID=""
fi

echo "5️⃣ Starting Battery Monitoring System..."

# Get absolute path to config file
BATTERY_CONFIG="$(pwd)/src/mtrx3760_battery/config/battery_params.yaml"

if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "   Using real battery data from /battery_state"
    "$SCRIPT_DIR/start_battery_clean.sh" "$(pwd)" "battery_monitor_node" "$BATTERY_CONFIG" > /tmp/battery_monitor.log 2>&1 &
    BATTERY_PID=$!
else
    echo "   Using battery simulator for Gazebo"
    "$SCRIPT_DIR/start_battery_clean.sh" "$(pwd)" "battery_simulator_node" "$BATTERY_CONFIG" > /tmp/battery_simulator.log 2>&1 &
    BATTERY_PID=$!
fi

echo "   Starting battery terminal display in new window..."
# Try to open in a new terminal window
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "source $(pwd)/install/setup.bash && ros2 run mtrx3760_battery battery_terminal_display --ros-args --params-file '$BATTERY_CONFIG'; exec bash" &
    BATTERY_DISPLAY_PID=$!
elif command -v xterm &> /dev/null; then
    xterm -e "source $(pwd)/install/setup.bash && ros2 run mtrx3760_battery battery_terminal_display --ros-args --params-file '$BATTERY_CONFIG'; bash" &
    BATTERY_DISPLAY_PID=$!
else
    echo "   ⚠️  No terminal emulator found, running in background"
    ros2 run mtrx3760_battery battery_terminal_display --ros-args --params-file "$BATTERY_CONFIG" > /tmp/battery_display.log 2>&1 &
    BATTERY_DISPLAY_PID=$!
fi
sleep 2

echo "6️⃣ Launching RViz2 with SLAM Toolbox config (output redirected to /tmp/rviz.log)..."
# Try to find RViz config in install directory first, then source
RVIZ_CONFIG="$(pwd)/install/warehouse_robot_system/share/warehouse_robot_system/config/slam_toolbox.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$(pwd)/src/turtlebot3_simulations/warehouse_robot_system/config/slam_toolbox.rviz"
fi

if [ -f "$RVIZ_CONFIG" ]; then
    echo "   Using RViz config: $RVIZ_CONFIG"
    rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz.log 2>&1 &
else
    echo "   ⚠️  RViz config not found, using default"
    rviz2 > /tmp/rviz.log 2>&1 &
fi
RVIZ_PID=$!
sleep 3

if [ "$PRELOAD_MAP" = false ]; then
    echo "7️⃣ Starting Autonomous SLAM Exploration Controller..."
    echo "⏳ Waiting for SLAM to be ready..."
    sleep 3

    # Check if map topic is available before starting autonomous controller
    echo "🔍 Checking for /map topic..."
    timeout 10s bash -c 'until ros2 topic list | grep -q "^/map$"; do sleep 1; done' || {
        echo "⚠️  Map topic not available yet, but starting controller anyway..."
    }

    # Start the new autonomous exploration node using clean launcher to avoid conda conflicts
    "$SCRIPT_DIR/start_autonomous_slam_clean.sh" "$(pwd)" &
    SLAM_PID=$!
    sleep 3
else
    echo "7️⃣ Skipping Autonomous SLAM Controller (preload mode)"
    SLAM_PID=""
    sleep 2
fi

echo ""
if [ "$PRELOAD_MAP" = true ]; then
    echo "✅ Warehouse System Started (Preload Mode)!"
else
    echo "✅ Autonomous SLAM System Started!"
fi
echo ""
echo "📊 Running Components:"
if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "   🤖 Physical TurtleBot3 (hardware bringup on robot)"
else
    echo "   🔧 robot_state_publisher (PID: $RSP_PID)"
    echo "   🤖 TurtleBot3 in Gazebo (PID: $SPAWN_PID)"
fi
if [ "$PRELOAD_MAP" = true ]; then
    echo "   🗺️  SLAM Toolbox - Localization (PID: $SLAM_TOOLBOX_PID)"
else
    echo "   🗺️  SLAM Toolbox - Mapping (PID: $SLAM_TOOLBOX_PID)"
fi
if [ ! -z "$ROSBRIDGE_PID" ]; then
    echo "   🌐 rosbridge WebSocket (PID: $ROSBRIDGE_PID)"
fi
if [ ! -z "$WEB_SERVER_PID" ]; then
    echo "   🌐 Web Server (PID: $WEB_SERVER_PID)"
fi
echo "   🔋 Battery Monitor (PID: $BATTERY_PID)"
echo "   📊 Battery Display (PID: $BATTERY_DISPLAY_PID)"
echo "   🖥️  RViz2 (PID: $RVIZ_PID)"
if [ "$PRELOAD_MAP" = false ]; then
    echo "   🧠 Autonomous SLAM Controller (PID: $SLAM_PID)"
fi
echo ""
echo "🎯 Mode: $([ "$USE_PHYSICAL_ROBOT" = true ] && echo "Physical Robot" || echo "Simulation")"
echo "   use_sim_time: $USE_SIM_TIME"
echo ""
if [ "$PRELOAD_MAP" = false ]; then
    echo "🎯 System Behavior:"
    echo "   • Robot will automatically explore the environment"
    echo "   • Uses Expanding Wavefront Frontier Detection"
    echo "   • Plans optimal paths using A* with cost map"
    echo "   • Follows paths using Pure Pursuit algorithm"
    echo "   • Avoids obstacles dynamically using local costmap"
    echo "   • Saves map automatically when exploration is complete"
else
    echo "🎯 System Behavior:"
    echo "   • Pre-loaded map from previous exploration"
    echo "   • SLAM Toolbox in localization mode"
    echo "   • Robot localized on existing map"
    echo "   • Ready for delivery or inspection operations"
    echo "   • Skipping autonomous exploration phase"
fi
echo ""
echo "🖥️  RViz2 Setup:"
echo "   • RViz2 opened with SLAM Toolbox configuration"
echo "   • Map, LaserScan, and Path displays are pre-configured"
echo "   • Fixed Frame: 'map' (may show error initially until SLAM starts)"
echo "   • Wait 10-20 seconds for SLAM Toolbox to initialize"
echo ""
echo "📈 Monitoring:"
echo "   • Watch RViz to see autonomous exploration"
echo "   • Robot will move to frontiers automatically"
echo "   • SLAM builds map as robot explores"
echo "   • Battery status shown in separate terminal window"
if [ ! -z "$WEB_SERVER_PID" ]; then
    echo "   • 🌐 Web dashboard: http://localhost:3000 (RUNNING)"
elif [ ! -z "$ROSBRIDGE_PID" ]; then
    echo "   • Web dashboard: Open browser to http://localhost:3000"
    echo "     (or run: cd turtlebot3_ws/src/mtrx3760_battery/web && npm run dev)"
else
    echo "   • Web dashboard: Install rosbridge to enable"
fi
echo "   • System logs show state transitions and progress"
echo "   • SLAM Toolbox logs: tail -f /tmp/slam_toolbox.log"
echo "   • Battery logs: tail -f /tmp/battery_monitor.log (or battery_simulator.log)"
if [ ! -z "$ROSBRIDGE_PID" ]; then
    echo "   • rosbridge logs: tail -f /tmp/rosbridge.log"
fi
if [ ! -z "$WEB_SERVER_PID" ]; then
    echo "   • Web server logs: tail -f /tmp/battery_web.log"
fi
echo "   • RViz logs: tail -f /tmp/rviz.log"
echo ""
echo "🔄 Exploration Process:"
echo "   1. Detect frontiers (unexplored areas)"
echo "   2. Select best frontier (size + distance)"
echo "   3. Plan path using A* pathfinding"
echo "   4. Follow path with Pure Pursuit"
echo "   5. Repeat until no frontiers remain"
echo ""
echo "💡 The robot is now fully autonomous!"
echo "   No manual control needed - it will explore and map automatically."
echo ""
echo "📊 Visualization Topics (add in RViz):"
echo "   • /map - SLAM-generated map"
echo "   • /scan - Laser scan data"
echo "   • /exploration/path - Planned exploration path"
echo "   • /exploration/frontier_cells - Detected frontiers (debug mode)"
echo ""
echo "Press Ctrl+C to stop all components"

# Wait for user interrupt or exploration completion
EXPLORATION_COMPLETE=false

if [ "$PRELOAD_MAP" = true ]; then
    # In preload mode, go directly to mode selection
    echo ""
    echo "🗺️  Map loaded successfully!"
    echo "   Skipping exploration phase..."
    echo ""
    sleep 2
    
    EXPLORATION_COMPLETE=true
    touch /tmp/slam_exploration_complete.marker
    offer_mode_selection
else
    # Normal exploration mode
    echo ""
    echo "⏳ Monitoring exploration progress..."
    echo "   (Exploration will complete automatically when map is finished)"
    echo ""

    while true; do
        # Check if any critical process died
        if ! ps -p $SLAM_PID > /dev/null 2>&1; then
            # Wait a moment to ensure clean shutdown
            sleep 2
            
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ AUTONOMOUS SLAM CONTROLLER COMPLETED!"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo ""
            echo "   ✓ Exploration finished"
            echo "   ✓ Robot returned home"
            echo "   ✓ Map saved successfully"
            echo ""
            EXPLORATION_COMPLETE=true
            break
        fi
        
        if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
            echo "❌ SLAM Toolbox process died!"
            break
        fi
        
        if ! ps -p $BATTERY_PID > /dev/null 2>&1; then
            echo "⚠️  Battery Monitor process died (non-critical)"
        fi
        
        # Only check RSP if in simulation mode
        if [ "$USE_PHYSICAL_ROBOT" = false ] && [ ! -z "$RSP_PID" ]; then
            if ! ps -p $RSP_PID > /dev/null 2>&1; then
                echo "❌ robot_state_publisher process died!"
                break
            fi
        fi
        
        sleep 1
    done

    # If exploration completed successfully, offer to switch to delivery mode
    if [ "$EXPLORATION_COMPLETE" = true ]; then
        # Create marker file for cleanup function
        touch /tmp/slam_exploration_complete.marker
        # Call the mode selection function
        offer_mode_selection
    fi
fi

cleanup