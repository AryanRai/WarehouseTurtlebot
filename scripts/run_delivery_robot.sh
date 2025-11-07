#!/bin/bash
# Delivery Robot System Runner
# Runs the delivery robot with localization mode (requires pre-existing map)
#
# Usage:
#   ./scripts/run_delivery_robot.sh          # Normal mode
#   ./scripts/run_delivery_robot.sh -web     # With web dashboard

# Deactivate conda if active to prevent library conflicts
if [ ! -z "$CONDA_PREFIX" ]; then
    echo " Deactivating conda environment..."
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
    
    echo "    Conda deactivated"
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
            echo "  -web, --web    Start web dashboard"
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

echo " Starting Delivery Robot System"
echo "=================================="
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
    echo " Workspace not built! Please run './scripts/build_project.sh' first."
    exit 1
fi

source install/setup.bash

# Check for and kill any existing rosbridge instances
EXISTING_ROSBRIDGE=$(pgrep -f "rosbridge_websocket" | tr '\n' ' ')
if [ ! -z "$EXISTING_ROSBRIDGE" ]; then
    echo " Found existing rosbridge instance(s): $EXISTING_ROSBRIDGE"
    echo "   Cleaning up old rosbridge processes..."
    
    for pid in $EXISTING_ROSBRIDGE; do
        if ps -p $pid > /dev/null 2>&1; then
            kill -TERM $pid 2>/dev/null
        fi
    done
    
    sleep 2
    
    for pid in $EXISTING_ROSBRIDGE; do
        if ps -p $pid > /dev/null 2>&1; then
            kill -9 $pid 2>/dev/null
        fi
    done
    
    echo "    Old rosbridge instances cleaned up"
    echo ""
fi

# Check if Gazebo is running
USE_PHYSICAL_ROBOT=false
if ! pgrep -f "gz sim" > /dev/null; then
    echo "️  Gazebo is not running!"
    echo ""
    echo "Would you like to run on physical TurtleBot3 instead? (y/n)"
    read -r response
    
    if [[ "$response" =~ ^[Yy]$ ]]; then
        USE_PHYSICAL_ROBOT=true
        echo ""
        echo " Switching to Physical TurtleBot3 Mode"
        echo "========================================"
        echo ""
        echo "Prerequisites:"
        echo "1. TurtleBot3 must be powered on and connected"
        echo "2. Hardware bringup must be running on TurtleBot"
        echo ""
        echo "Press Enter to continue or Ctrl+C to cancel..."
        read -r
    else
        echo ""
        echo " Please start Gazebo first with:"
        echo "   ./launch_mgen.sh"
        exit 1
    fi
else
    echo " Gazebo is running - using simulation mode"
    echo ""
fi

# Function to cleanup processes on exit
cleanup() {
    echo ""
    echo " Shutting down Delivery Robot System..."
    
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
    
    if [ ! -z "$ROSBRIDGE_PID" ] && ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
        echo "   Stopping rosbridge..."
        kill -TERM $ROSBRIDGE_PID 2>/dev/null
        for i in {1..6}; do
            if ! ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
                break
            fi
            sleep 0.5
        done
        if ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
            kill -9 $ROSBRIDGE_PID 2>/dev/null
        fi
    fi
    
    if [ ! -z "$WEB_SERVER_PID" ] && ps -p $WEB_SERVER_PID > /dev/null 2>&1; then
        echo "   Stopping web server..."
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
    
    if [ ! -z "$DELIVERY_PID" ] && ps -p $DELIVERY_PID > /dev/null 2>&1; then
        echo "   Stopping Delivery Robot..."
        kill -TERM $DELIVERY_PID 2>/dev/null
        sleep 0.5
    fi
    
    if [ ! -z "$SPAWN_PID" ] && ps -p $SPAWN_PID > /dev/null 2>&1; then
        echo "   Stopping TurtleBot3 spawn..."
        kill -TERM $SPAWN_PID 2>/dev/null
        sleep 0.3
    fi
    
    # Final cleanup
    echo "   Cleaning up any remaining processes..."
    for pid in $RSP_PID $SLAM_TOOLBOX_PID $BATTERY_PID $ROSBRIDGE_PID $WEB_SERVER_PID $RVIZ_PID $DELIVERY_PID $SPAWN_PID; do
        if [ ! -z "$pid" ] && ps -p $pid > /dev/null 2>&1; then
            kill -9 $pid 2>/dev/null
        fi
    done
    
    echo " Delivery Robot system stopped."
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

echo " Starting Delivery Robot Components..."
echo ""

# Different startup for physical robot vs simulation
if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo " Physical TurtleBot3 Mode"
    echo "==========================="
    echo ""
    USE_SIM_TIME="False"
    RSP_PID=""
    SPAWN_PID=""
else
    echo " Simulation Mode"
    echo "=================="
    echo ""
    
    echo "1️⃣ Starting robot_state_publisher..."
    ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=True &
    RSP_PID=$!
    sleep 2

    echo "2️⃣ Spawning TurtleBot3 at origin (0, 0)..."
    ros2 launch turtlebot3_gazebo spawn_turtlebot3.launch.py x_pose:=0.0 y_pose:=0.0 &
    SPAWN_PID=$!
    sleep 4

    echo " Stopping wall-following node..."
    pkill -f turtlebot3_drive_node
    sleep 1

    echo " Robot spawned - ready for delivery operations"
    
    USE_SIM_TIME="True"
fi

echo "3️⃣ Starting SLAM Toolbox in LOCALIZATION mode..."
echo "   (Make sure you have a saved map!)"
ros2 launch slam_toolbox localization_launch.py use_sim_time:=$USE_SIM_TIME > /tmp/slam_toolbox.log 2>&1 &
SLAM_TOOLBOX_PID=$!

echo " Waiting for SLAM Toolbox to initialize..."
sleep 5

if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
    echo " SLAM Toolbox failed to start!"
    cleanup
    exit 1
fi

echo "4️⃣ Starting rosbridge for web dashboard..."
if ros2 pkg list 2>/dev/null | grep -q "rosbridge_server" 2>/dev/null; then
    "$SCRIPT_DIR/start_rosbridge_clean.sh" "$(pwd)" > /tmp/rosbridge.log 2>&1 &
    ROSBRIDGE_PID=$!
    sleep 3
    
    if ps -p $ROSBRIDGE_PID > /dev/null 2>&1; then
        echo "    rosbridge started (PID: $ROSBRIDGE_PID)"
    else
        echo "   ️  rosbridge failed to start"
        ROSBRIDGE_PID=""
    fi
else
    echo "   ️  rosbridge_server not installed"
    ROSBRIDGE_PID=""
fi

echo "5️⃣ Starting Battery Monitoring System..."
BATTERY_CONFIG="$(pwd)/src/mtrx3760_battery/config/battery_params.yaml"

if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "   Using real battery data"
    "$SCRIPT_DIR/start_battery_clean.sh" "$(pwd)" "battery_monitor_node" "$BATTERY_CONFIG" > /tmp/battery_monitor.log 2>&1 &
    BATTERY_PID=$!
else
    echo "   Using battery simulator"
    "$SCRIPT_DIR/start_battery_clean.sh" "$(pwd)" "battery_simulator_node" "$BATTERY_CONFIG" > /tmp/battery_simulator.log 2>&1 &
    BATTERY_PID=$!
fi

echo "6️⃣ Launching RViz2..."
RVIZ_CONFIG="$(pwd)/install/warehouse_robot_system/share/warehouse_robot_system/config/slam_toolbox.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="$(pwd)/src/turtlebot3_simulations/warehouse_robot_system/config/slam_toolbox.rviz"
fi

if [ -f "$RVIZ_CONFIG" ]; then
    echo "   Using RViz config: $RVIZ_CONFIG"
    rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz.log 2>&1 &
else
    echo "   Using default RViz config"
    rviz2 > /tmp/rviz.log 2>&1 &
fi
RVIZ_PID=$!
sleep 3

echo "7️⃣ Starting Delivery Robot Node..."
echo " Waiting for SLAM to be ready..."
sleep 3

ros2 run warehouse_robot_system delivery_robot_node &
DELIVERY_PID=$!
sleep 3

echo ""
echo " Delivery Robot System Started!"
echo ""
echo " Running Components:"
if [ "$USE_PHYSICAL_ROBOT" = true ]; then
    echo "    Physical TurtleBot3"
else
    echo "    robot_state_publisher (PID: $RSP_PID)"
    echo "    TurtleBot3 in Gazebo (PID: $SPAWN_PID)"
fi
echo "   ️  SLAM Toolbox - Localization (PID: $SLAM_TOOLBOX_PID)"
if [ ! -z "$ROSBRIDGE_PID" ]; then
    echo "    rosbridge WebSocket (PID: $ROSBRIDGE_PID)"
fi
echo "    Battery Monitor (PID: $BATTERY_PID)"
echo "   ️  RViz2 (PID: $RVIZ_PID)"
echo "    Delivery Robot (PID: $DELIVERY_PID)"
echo ""
echo " Mode: $([ "$USE_PHYSICAL_ROBOT" = true ] && echo "Physical Robot" || echo "Simulation")"
echo "   use_sim_time: $USE_SIM_TIME"
echo ""
echo " Delivery Robot Workflow:"
echo "================================"
echo ""
echo "STEP 1: Define Delivery Zones"
echo "   • Use RViz 'Publish Point' tool (toolbar)"
echo "   • Click on the map to add delivery zones"
echo "   • Each click creates a new zone (Zone_1, Zone_2, etc.)"
echo ""
echo "STEP 2: Save Delivery Zones"
echo "   Run in another terminal:"
echo "   $ cd turtlebot3_ws"
echo "   $ source install/setup.bash"
echo "   $ ros2 service call /save_delivery_zones std_srvs/srv/Trigger"
echo ""
echo "   This saves zones to: delivery_zones.yaml"
echo ""
echo "STEP 3: Add Delivery Requests"
echo "   Edit: src/turtlebot3_simulations/warehouse_robot_system/src/delivery_robot_node.cpp"
echo "   Uncomment the example delivery requests (lines ~50-60)"
echo "   Or add your own requests programmatically"
echo "   Then rebuild: colcon build --packages-select warehouse_robot_system"
echo ""
echo "STEP 4: Start Deliveries"
echo "   Run in another terminal:"
echo "   $ ros2 service call /start_deliveries std_srvs/srv/Trigger"
echo ""
echo "   The robot will:"
echo "   • Calculate optimal route (TSP)"
echo "   • Navigate to each zone"
echo "   • Log deliveries to: delivery_log.csv"
echo ""
echo " Monitoring:"
echo "   • Watch RViz to see robot navigation"
echo "   • Check delivery status: ros2 topic echo /delivery/status"
echo "   • View delivery log: cat delivery_log.csv"
echo "   • SLAM logs: tail -f /tmp/slam_toolbox.log"
echo ""
echo " Output Files:"
echo "   • delivery_zones.yaml - Saved delivery zones"
echo "   • delivery_log.csv - Delivery records with timestamps"
echo ""
echo " Quick Commands:"
echo "   Save zones:  ros2 service call /save_delivery_zones std_srvs/srv/Trigger"
echo "   Start:       ros2 service call /start_deliveries std_srvs/srv/Trigger"
echo "   Status:      ros2 topic echo /delivery/status"
echo ""
echo "Press Ctrl+C to stop all components"

# Wait for user interrupt
while true; do
    if ! ps -p $DELIVERY_PID > /dev/null 2>&1; then
        echo " Delivery Robot process died!"
        break
    fi
    
    if ! ps -p $SLAM_TOOLBOX_PID > /dev/null 2>&1; then
        echo " SLAM Toolbox process died!"
        break
    fi
    
    if [ "$USE_PHYSICAL_ROBOT" = false ] && [ ! -z "$RSP_PID" ]; then
        if ! ps -p $RSP_PID > /dev/null 2>&1; then
            echo " robot_state_publisher process died!"
            break
        fi
    fi
    
    sleep 1
done

cleanup
