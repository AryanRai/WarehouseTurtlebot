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
while [[ $# -gt 0 ]]; do
    case $1 in
        -web|--web)
            START_WEB_DASHBOARD=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [-web]"
            echo ""
            echo "Options:"
            echo "  -web, --web    Start web dashboard (opens browser at http://localhost:3000)"
            echo "  -h, --help     Show this help message"
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

echo "🤖 Starting Autonomous SLAM System with SLAM Toolbox"
echo "====================================================="
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
if [ "$START_WEB_DASHBOARD" = true ]; then
    echo "   Web Dashboard: ENABLED"
fi
echo ""

# Save script directory before changing to workspace
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

cd "$(dirname "$0")/../turtlebot3_ws"

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built! Please run './scripts/build_project.sh' first."
    exit 1
fi

source install/setup.bash

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

# Function to cleanup processes on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down Autonomous SLAM System..."
    
    # Gracefully shutdown background processes
    if [ ! -z "$RSP_PID" ] && ps -p $RSP_PID > /dev/null 2>&1; then
        echo "   Stopping robot_state_publisher..."
        kill -TERM $RSP_PID 2>/dev/null
        sleep 0.5
    fi
    
    if [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
        echo "   Stopping SLAM Toolbox..."
        kill -TERM $SLAM_TOOLBOX_PID 2>/dev/null
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

echo "3️⃣ Starting SLAM Toolbox (output redirected to /tmp/slam_toolbox.log)..."
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=$USE_SIM_TIME > /tmp/slam_toolbox.log 2>&1 &
SLAM_TOOLBOX_PID=$!

echo "⏳ Waiting for SLAM Toolbox to initialize..."
sleep 5

# Check if SLAM Toolbox started successfully
if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
    echo "❌ SLAM Toolbox failed to start!"
    cleanup
    exit 1
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

echo ""
echo "✅ Autonomous SLAM System Started!"
echo ""
echo "📊 Running Components:"
if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "   🤖 Physical TurtleBot3 (hardware bringup on robot)"
else
    echo "   🔧 robot_state_publisher (PID: $RSP_PID)"
    echo "   🤖 TurtleBot3 in Gazebo (PID: $SPAWN_PID)"
fi
echo "   🗺️  SLAM Toolbox (PID: $SLAM_TOOLBOX_PID)"
if [ ! -z "$ROSBRIDGE_PID" ]; then
    echo "   🌐 rosbridge WebSocket (PID: $ROSBRIDGE_PID)"
fi
if [ ! -z "$WEB_SERVER_PID" ]; then
    echo "   🌐 Web Server (PID: $WEB_SERVER_PID)"
fi
echo "   🔋 Battery Monitor (PID: $BATTERY_PID)"
echo "   📊 Battery Display (PID: $BATTERY_DISPLAY_PID)"
echo "   🖥️  RViz2 (PID: $RVIZ_PID)"
echo "   🧠 Autonomous SLAM Controller (PID: $SLAM_PID)"
echo ""
echo "🎯 Mode: $([ "$USE_PHYSICAL_ROBOT" = true ] && echo "Physical Robot" || echo "Simulation")"
echo "   use_sim_time: $USE_SIM_TIME"
echo ""
echo "🎯 System Behavior:"
echo "   • Robot will automatically explore the environment"
echo "   • Uses Expanding Wavefront Frontier Detection"
echo "   • Plans optimal paths using A* with cost map"
echo "   • Follows paths using Pure Pursuit algorithm"
echo "   • Avoids obstacles dynamically using local costmap"
echo "   • Saves map automatically when exploration is complete"
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
    echo "   [1] 📦 DELIVERY MODE"
    echo "       • Multi-point delivery operations"
    echo "       • Define zones via RViz clicks"
    echo "       • Route optimization (TSP)"
    echo "       • Delivery logging to CSV"
    echo ""
    echo "   [2] 🔍 INSPECTION MODE (Coming Soon)"
    echo "       • Damage detection with camera"
    echo "       • Inspection point navigation"
    echo "       • Damage report generation"
    echo ""
    echo "   [3] ❌ EXIT"
    echo "       • Shutdown system"
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "   Enter your choice [1/2/3]"
    echo "   (You have 60 seconds to respond)"
    echo ""
    echo -n "   👉 Your choice: "
    read -r -t 60 response || response="3"
    echo ""
    
    if [[ "$response" == "1" ]]; then
        echo ""
        echo "🔄 Switching to Delivery Mode..."
        echo "================================"
        echo ""
        
        # Stop SLAM Toolbox (mapping mode)
        if [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
            echo "   Stopping SLAM Toolbox (mapping mode)..."
            kill -TERM $SLAM_TOOLBOX_PID 2>/dev/null
            sleep 2
        fi
        
        # Start SLAM Toolbox in localization mode
        echo "   Starting SLAM Toolbox (localization mode)..."
        ros2 launch slam_toolbox localization_launch.py use_sim_time:=$USE_SIM_TIME > /tmp/slam_toolbox.log 2>&1 &
        SLAM_TOOLBOX_PID=$!
        sleep 5
        
        if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
            echo "   ❌ Failed to start SLAM in localization mode"
            cleanup
            exit 1
        fi
        
        echo "   ✅ SLAM Toolbox running in localization mode"
        
        # Start delivery robot node
        echo "   Starting Delivery Robot node..."
        ros2 run warehouse_robot_system delivery_robot_node &
        DELIVERY_PID=$!
        sleep 3
        
        if ! ps -p $DELIVERY_PID > /dev/null 2>&1; then
            echo "   ❌ Failed to start delivery robot"
            cleanup
            exit 1
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
        
        # Wait in delivery mode
        while true; do
            if ! ps -p $DELIVERY_PID > /dev/null 2>&1; then
                echo "❌ Delivery Robot process died!"
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
        
    elif [[ "$response" == "2" ]]; then
        echo ""
        echo "🔄 Switching to Inspection Mode..."
        echo "================================"
        echo ""
        
        # Stop SLAM Toolbox (mapping mode)
        if [ ! -z "$SLAM_TOOLBOX_PID" ] && ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
            echo "   Stopping SLAM Toolbox (mapping mode)..."
            kill -TERM $SLAM_TOOLBOX_PID 2>/dev/null
            sleep 2
        fi
        
        # Start SLAM Toolbox in localization mode
        echo "   Starting SLAM Toolbox (localization mode)..."
        ros2 launch slam_toolbox localization_launch.py use_sim_time:=$USE_SIM_TIME > /tmp/slam_toolbox.log 2>&1 &
        SLAM_TOOLBOX_PID=$!
        sleep 5
        
        if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
            echo "   ❌ Failed to start SLAM in localization mode"
            cleanup
            exit 1
        fi
        
        echo "   ✅ SLAM Toolbox running in localization mode"
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "🔍 INSPECTION MODE"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "   ⚠️  Inspection mode is not yet implemented!"
        echo ""
        echo "   Planned features:"
        echo "   • Camera-based damage detection"
        echo "   • Navigate to inspection points"
        echo "   • Save damage reports to disk"
        echo "   • Image capture and logging"
        echo ""
        echo "   For now, the system will remain in localization mode."
        echo "   You can manually control the robot or press Ctrl+C to exit."
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "Press Ctrl+C to stop system"
        echo ""
        
        # Wait in inspection mode (placeholder)
        while true; do
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
        
    else
        echo ""
        echo "   Exiting - shutting down system..."
    fi
fi

cleanup