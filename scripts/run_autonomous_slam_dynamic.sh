#!/bin/bash

# ============================================================================
# MTRX3760 Project 2 - Dynamic Autonomous SLAM Script
# Description: Polymorphic warehouse robot control using dynamic node switching.
#              Replaces individual robot node launches with unified polymorphic 
#              control. Provides all functionality of run_autonomous_slam.sh
#              with runtime mode switching capability.
# Author(s): Dylan George
# Last Edited: 2025-11-05
# ============================================================================

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_DIR="$HOME/Desktop/Github/MTRX3760_Project_2/turtlebot3_ws"
MAP_DIR="$WORKSPACE_DIR/src/turtlebot3_simulations/warehouse_robot_system/maps"
MANAGER_NODE_NAME="polymorphic_warehouse_manager"

# Global variables
WAREHOUSE_MANAGER_PID=""
SLAM_PID=""
NAVIGATION_PID=""
CURRENT_MODE="none"

echo -e "${CYAN}============================================================================${NC}"
echo -e "${CYAN}           POLYMORPHIC WAREHOUSE MANAGER - MTRX3760 Project 2${NC}"
echo -e "${CYAN}            Dynamic Robot Control with Runtime Mode Switching${NC}"
echo -e "${CYAN}============================================================================${NC}"
echo ""

# Function to check if TurtleBot3 simulation is running
check_simulation() {
    if ! pgrep -f "gzserver" > /dev/null; then
        echo -e "${RED} TurtleBot3 simulation not detected!${NC}"
        echo -e "${YELLOW}Please run TurtleBot3 Gazebo simulation first:${NC}"
        echo "   export TURTLEBOT3_MODEL=waffle"
        echo "   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
        echo ""
        read -p "Continue anyway? (y/N): " -r
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    else
        echo -e "${GREEN} TurtleBot3 simulation detected${NC}"
    fi
}

# Function to start SLAM
start_slam() {
    echo -e "${BLUE}️ Starting SLAM...${NC}"
    
    cd "$WORKSPACE_DIR"
    source install/setup.bash
    
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True &
    SLAM_PID=$!
    
    sleep 3
    
    if kill -0 $SLAM_PID 2>/dev/null; then
        echo -e "${GREEN} SLAM running (PID: $SLAM_PID)${NC}"
    else
        echo -e "${RED} Failed to start SLAM${NC}"
        exit 1
    fi
}

# Function to start navigation
start_navigation() {
    local map_file=$1
    
    if [ ! -f "$map_file" ]; then
        echo -e "${RED} Map file not found: $map_file${NC}"
        return 1
    fi
    
    echo -e "${BLUE} Starting Navigation with map: $(basename "$map_file")${NC}"
    
    cd "$WORKSPACE_DIR" 
    source install/setup.bash
    
    ros2 launch turtlebot3_navigation2 navigation2.launch.py \
        use_sim_time:=True \
        map:="$map_file" &
    NAVIGATION_PID=$!
    
    sleep 3
    
    if kill -0 $NAVIGATION_PID 2>/dev/null; then
        echo -e "${GREEN} Navigation running (PID: $NAVIGATION_PID)${NC}"
        return 0
    else
        echo -e "${RED} Failed to start Navigation${NC}"
        return 1
    fi
}

# Function to start polymorphic warehouse manager
start_warehouse_manager() {
    local initial_mode=${1:-"none"}
    
    echo -e "${PURPLE} Starting Polymorphic Warehouse Manager...${NC}"
    echo -e "${PURPLE}   Mode: $initial_mode${NC}"
    
    cd "$WORKSPACE_DIR"
    source install/setup.bash
    
    # Start the polymorphic warehouse manager node
    ros2 run warehouse_robot_system warehouse_manager_node \
        --ros-args -p robot_mode:="$initial_mode" &
    
    WAREHOUSE_MANAGER_PID=$!
    
    # Wait for node to initialize
    sleep 4
    
    # Check if node is running
    if kill -0 $WAREHOUSE_MANAGER_PID 2>/dev/null; then
        echo -e "${GREEN} Polymorphic Warehouse Manager running (PID: $WAREHOUSE_MANAGER_PID)${NC}"
        CURRENT_MODE="$initial_mode"
        return 0
    else
        echo -e "${RED} Failed to start Polymorphic Warehouse Manager${NC}"
        return 1
    fi
}

# Function to switch robot mode using polymorphism
switch_mode() {
    local mode=$1
    echo -e "${PURPLE} Polymorphic mode switch: $CURRENT_MODE → $mode${NC}"
    
    ros2 param set /$MANAGER_NODE_NAME robot_mode "$mode" 2>/dev/null
    
    if [ $? -eq 0 ]; then
        CURRENT_MODE="$mode"
        echo -e "${GREEN} Polymorphic switch complete${NC}"
        
        # Give some time for the switch to complete
        sleep 2
        
        # Show polymorphic architecture info
        echo -e "${CYAN} POLYMORPHIC ARCHITECTURE ACTIVE:${NC}"
        echo -e "   Base pointer: ${YELLOW}WarehouseRobot*${NC}"
        case $mode in
            "inspection")
                echo -e "   Derived type: ${GREEN}InspectionRobot${NC}"
                echo -e "   Virtual calls: ${GREEN}→ inspection methods${NC}"
                ;;
            "delivery")
                echo -e "   Derived type: ${BLUE}DeliveryRobot${NC}"
                echo -e "   Virtual calls: ${BLUE}→ delivery methods${NC}"
                ;;
            "manual")
                echo -e "   Mode: ${YELLOW}Manual Control${NC}"
                ;;
        esac
    else
        echo -e "${RED} Failed to switch mode${NC}"
    fi
}

# Function to start operations polymorphically
start_operations() {
    echo -e "${GREEN}️ Starting polymorphic operations...${NC}"
    
    local result=$(ros2 service call /warehouse/start_operations std_srvs/srv/Trigger 2>/dev/null)
    
    if echo "$result" | grep -q "success: true"; then
        echo -e "${GREEN} Polymorphic operations started${NC}"
        echo -e "${CYAN} Virtual function call: mActiveRobot->startOperations()${NC}"
    else
        echo -e "${RED} Failed to start operations${NC}"
        echo "Response: $result"
    fi
}

# Function to stop operations polymorphically  
stop_operations() {
    echo -e "${YELLOW}️ Stopping polymorphic operations...${NC}"
    
    local result=$(ros2 service call /warehouse/stop_operations std_srvs/srv/Trigger 2>/dev/null)
    
    if echo "$result" | grep -q "success: true"; then
        echo -e "${GREEN} Polymorphic operations stopped${NC}"
        echo -e "${CYAN} Virtual function call: mActiveRobot->stopOperations()${NC}"
    else
        echo -e "${RED} Failed to stop operations${NC}"
    fi
}

# Function to get status polymorphically
get_status() {
    echo -e "${BLUE} Getting polymorphic status...${NC}"
    
    local result=$(ros2 service call /warehouse/get_status std_srvs/srv/Trigger 2>/dev/null)
    
    if echo "$result" | grep -q "success: true"; then
        local message=$(echo "$result" | grep "message:" | cut -d"'" -f2)
        echo -e "${GREEN}Status: $message${NC}"
    else
        echo -e "${RED} Failed to get status${NC}"
    fi
}

# Function to save map
save_map() {
    local map_name=$1
    
    if [ -z "$map_name" ]; then
        echo -n "Enter map name: "
        read map_name
    fi
    
    if [ -z "$map_name" ]; then
        echo -e "${RED} Map name cannot be empty${NC}"
        return 1
    fi
    
    echo -e "${BLUE} Saving map: $map_name${NC}"
    
    cd "$MAP_DIR"
    ros2 run nav2_map_server map_saver_cli -f "$map_name"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN} Map saved: $MAP_DIR/${map_name}.yaml${NC}"
    else
        echo -e "${RED} Failed to save map${NC}"
    fi
}

# Function to list available maps
list_maps() {
    echo -e "${BLUE} Available maps:${NC}"
    
    if [ -d "$MAP_DIR" ]; then
        local maps=$(find "$MAP_DIR" -name "*.yaml" -exec basename {} .yaml \;)
        if [ -n "$maps" ]; then
            echo "$maps" | nl
        else
            echo -e "${YELLOW}No maps found in $MAP_DIR${NC}"
        fi
    else
        echo -e "${YELLOW}Map directory not found: $MAP_DIR${NC}"
    fi
}

# Function to select and load map
select_map() {
    list_maps
    echo ""
    echo -n "Enter map name (or number): "
    read map_choice
    
    if [[ "$map_choice" =~ ^[0-9]+$ ]]; then
        # User entered a number
        local map_list=($(find "$MAP_DIR" -name "*.yaml" -exec basename {} .yaml \;))
        local index=$((map_choice - 1))
        
        if [ $index -ge 0 ] && [ $index -lt ${#map_list[@]} ]; then
            map_choice="${map_list[$index]}"
        else
            echo -e "${RED} Invalid map number${NC}"
            return 1
        fi
    fi
    
    local map_file="$MAP_DIR/${map_choice}.yaml"
    
    if [ -f "$map_file" ]; then
        echo -e "${BLUE}️ Loading map: $map_choice${NC}"
        
        # Stop current navigation if running
        if [ ! -z "$NAVIGATION_PID" ] && kill -0 $NAVIGATION_PID 2>/dev/null; then
            echo -e "${YELLOW}Stopping current navigation...${NC}"
            kill $NAVIGATION_PID
            wait $NAVIGATION_PID 2>/dev/null || true
        fi
        
        # Start navigation with selected map
        start_navigation "$map_file"
    else
        echo -e "${RED} Map file not found: $map_file${NC}"
        return 1
    fi
}

# Interactive menu
show_menu() {
    echo ""
    echo -e "${CYAN}╔════════════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║                   POLYMORPHIC WAREHOUSE CONTROL                  ║${NC}"
    echo -e "${CYAN}╠════════════════════════════════════════════════════════════════════╣${NC}"
    echo -e "${CYAN}║${NC} Current Mode: ${GREEN}$CURRENT_MODE${NC} | SLAM: $([ ! -z "$SLAM_PID" ] && echo "${GREEN}ON${NC}" || echo "${RED}OFF${NC}") | Nav: $([ ! -z "$NAVIGATION_PID" ] && echo "${GREEN}ON${NC}" || echo "${RED}OFF${NC}")${CYAN} ║${NC}"
    echo -e "${CYAN}╚════════════════════════════════════════════════════════════════════╝${NC}"
    echo ""
    echo -e "${BOLD}SLAM & Navigation:${NC}"
    echo -e "  ${BLUE}[1]${NC} Basic SLAM Mode (mapping only)"
    echo -e "  ${BLUE}[2]${NC} Load Map & Start Navigation"
    echo ""
    echo -e "${BOLD}Polymorphic Robot Modes:${NC}"
    echo -e "  ${GREEN}[3]${NC} Inspection Mode ${PURPLE}(polymorphic: InspectionRobot)${NC}"
    echo -e "  ${BLUE}[4]${NC} Delivery Mode ${PURPLE}(polymorphic: DeliveryRobot)${NC}" 
    echo -e "  ${YELLOW}[5]${NC} Manual Control Mode"
    echo ""
    echo -e "${BOLD}Robot Operations (Polymorphic):${NC}"
    echo -e "  ${GREEN}[s]${NC} Start Operations ${PURPLE}(virtual call)${NC}"
    echo -e "  ${YELLOW}[x]${NC} Stop Operations ${PURPLE}(virtual call)${NC}"
    echo -e "  ${BLUE}[t]${NC} Get Status ${PURPLE}(virtual call)${NC}"
    echo ""
    echo -e "${BOLD}Map Management:${NC}"
    echo -e "  ${CYAN}[m]${NC} Save Current Map"
    echo -e "  ${CYAN}[l]${NC} List Available Maps"
    echo ""
    echo -e "${BOLD}System Control:${NC}"
    echo -e "  ${PURPLE}[h]${NC} Show this menu"
    echo -e "  ${RED}[q]${NC} Quit"
    echo ""
}

# Function to handle cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW} Cleaning up polymorphic warehouse system...${NC}"
    
    # Stop warehouse manager
    if [ ! -z "$WAREHOUSE_MANAGER_PID" ]; then
        echo -e "Stopping Polymorphic Warehouse Manager (PID: $WAREHOUSE_MANAGER_PID)..."
        kill $WAREHOUSE_MANAGER_PID 2>/dev/null || true
        wait $WAREHOUSE_MANAGER_PID 2>/dev/null || true
    fi
    
    # Stop SLAM
    if [ ! -z "$SLAM_PID" ]; then
        echo -e "Stopping SLAM (PID: $SLAM_PID)..."
        kill $SLAM_PID 2>/dev/null || true
        wait $SLAM_PID 2>/dev/null || true
    fi
    
    # Stop Navigation
    if [ ! -z "$NAVIGATION_PID" ]; then
        echo -e "Stopping Navigation (PID: $NAVIGATION_PID)..."
        kill $NAVIGATION_PID 2>/dev/null || true
        wait $NAVIGATION_PID 2>/dev/null || true
    fi
    
    echo -e "${GREEN} Cleanup complete${NC}"
    exit 0
}

# Set up signal handlers
trap cleanup SIGINT SIGTERM

# Main execution
main() {
    # Check simulation
    check_simulation
    
    # Build the workspace
    echo -e "${BLUE} Building polymorphic warehouse system...${NC}"
    cd "$WORKSPACE_DIR"
    
    # Try to build (might not work without ROS2 installed)
    if command -v colcon >/dev/null 2>&1; then
        colcon build --packages-select warehouse_robot_system
        
        if [ $? -ne 0 ]; then
            echo -e "${RED} Build failed!${NC}"
            exit 1
        fi
        
        echo -e "${GREEN} Build successful${NC}"
    else
        echo -e "${YELLOW}️ colcon not found, skipping build${NC}"
    fi
    
    # Start polymorphic warehouse manager
    if ! start_warehouse_manager "none"; then
        echo -e "${RED} Failed to start polymorphic warehouse manager${NC}"
        exit 1
    fi
    
    # Show initial menu
    show_menu
    
    # Interactive control loop
    while true; do
        echo -n -e "${PURPLE}polymorphic_warehouse>${NC} "
        read -r choice
        
        case $choice in
            1)
                # Stop navigation if running
                if [ ! -z "$NAVIGATION_PID" ] && kill -0 $NAVIGATION_PID 2>/dev/null; then
                    kill $NAVIGATION_PID
                    wait $NAVIGATION_PID 2>/dev/null || true
                    NAVIGATION_PID=""
                fi
                
                # Start SLAM if not running
                if [ -z "$SLAM_PID" ] || ! kill -0 $SLAM_PID 2>/dev/null; then
                    start_slam
                fi
                ;;
            2)
                # Stop SLAM if running
                if [ ! -z "$SLAM_PID" ] && kill -0 $SLAM_PID 2>/dev/null; then
                    echo -e "${YELLOW}Stopping SLAM for navigation...${NC}"
                    kill $SLAM_PID
                    wait $SLAM_PID 2>/dev/null || true
                    SLAM_PID=""
                fi
                
                select_map
                ;;
            3)
                switch_mode "inspection"
                ;;
            4)
                switch_mode "delivery"
                ;;
            5)
                switch_mode "manual"
                ;;
            s|S)
                start_operations
                ;;
            x|X)
                stop_operations
                ;;
            t|T)
                get_status
                ;;
            m|M)
                if [ ! -z "$SLAM_PID" ] && kill -0 $SLAM_PID 2>/dev/null; then
                    save_map
                else
                    echo -e "${RED} SLAM not running - cannot save map${NC}"
                fi
                ;;
            l|L)
                list_maps
                ;;
            h|H)
                show_menu
                ;;
            q|Q)
                cleanup
                ;;
            "")
                # Empty input, just continue
                ;;
            *)
                echo -e "${RED}Invalid choice: $choice${NC}"
                echo -e "Type ${PURPLE}'h'${NC} for menu or ${PURPLE}'q'${NC} to quit"
                ;;
        esac
    done
}

# Run main function
main "$@"