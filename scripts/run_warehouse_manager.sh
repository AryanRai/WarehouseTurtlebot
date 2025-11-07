#!/bin/bash

# ============================================================================
# MTRX3760 Project 2 - Warehouse Manager Launch Script
# Description: Dynamic warehouse robot control using WarehouseManager
#              Replaces individual robot node launches with unified control
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
NC='\033[0m' # No Color

# Configuration
WORKSPACE_DIR="$HOME/Desktop/Github/MTRX3760_Project_2/turtlebot3_ws"
MAP_DIR="$WORKSPACE_DIR/src/turtlebot3_simulations/warehouse_robot_system/maps"

echo -e "${CYAN}============================================================================${NC}"
echo -e "${CYAN}           DYNAMIC WAREHOUSE MANAGER - MTRX3760 Project 2${NC}"
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
        read -p "Press Enter to continue anyway or Ctrl+C to exit..."
    else
        echo -e "${GREEN} TurtleBot3 simulation detected${NC}"
    fi
}

# Function to start warehouse manager node
start_warehouse_manager() {
    local mode=$1
    
    echo -e "${BLUE} Starting Dynamic Warehouse Manager...${NC}"
    
    # Source the workspace
    cd "$WORKSPACE_DIR"
    source install/setup.bash
    
    # Set initial mode parameter and start node
    ros2 run warehouse_robot_system warehouse_manager_node \
        --ros-args -p robot_mode:="$mode" &
    
    WAREHOUSE_MANAGER_PID=$!
    
    # Wait for node to initialize
    sleep 3
    
    # Check if node is running
    if kill -0 $WAREHOUSE_MANAGER_PID 2>/dev/null; then
        echo -e "${GREEN} Warehouse Manager running (PID: $WAREHOUSE_MANAGER_PID)${NC}"
        return 0
    else
        echo -e "${RED} Failed to start Warehouse Manager${NC}"
        return 1
    fi
}

# Function to switch robot mode
switch_mode() {
    local mode=$1
    echo -e "${PURPLE} Switching to $mode mode...${NC}"
    
    ros2 param set /dynamic_warehouse_manager robot_mode "$mode"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN} Mode switched to $mode${NC}"
    else
        echo -e "${RED} Failed to switch mode${NC}"
    fi
}

# Function to start operations
start_operations() {
    echo -e "${GREEN}️ Starting robot operations...${NC}"
    
    ros2 service call /warehouse/start_operations std_srvs/srv/Trigger
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN} Operations started${NC}"
    else
        echo -e "${RED} Failed to start operations${NC}"
    fi
}

# Function to stop operations
stop_operations() {
    echo -e "${YELLOW}️ Stopping robot operations...${NC}"
    
    ros2 service call /warehouse/stop_operations std_srvs/srv/Trigger
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN} Operations stopped${NC}"
    else
        echo -e "${RED} Failed to stop operations${NC}"
    fi
}

# Interactive menu
show_menu() {
    echo ""
    echo -e "${CYAN} DYNAMIC WAREHOUSE CONTROL MENU${NC}"
    echo -e "${CYAN}=================================${NC}"
    echo -e "Current Mode: ${GREEN}$(ros2 param get /dynamic_warehouse_manager robot_mode 2>/dev/null | cut -d' ' -f2 || echo "unknown")${NC}"
    echo ""
    echo "Mode Selection:"
    echo -e "  ${BLUE}[1]${NC} Inspection Mode (with camera system)"
    echo -e "  ${BLUE}[2]${NC} Delivery Mode"
    echo -e "  ${BLUE}[3]${NC} Manual Control Mode"
    echo ""
    echo "Operations Control:"
    echo -e "  ${GREEN}[s]${NC} Start operations"
    echo -e "  ${YELLOW}[x]${NC} Stop operations"
    echo ""
    echo "System Control:"
    echo -e "  ${PURPLE}[m]${NC} Show this menu"
    echo -e "  ${RED}[q]${NC} Quit"
    echo ""
}

# Function to handle cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW} Cleaning up...${NC}"
    
    if [ ! -z "$WAREHOUSE_MANAGER_PID" ]; then
        echo -e "Stopping Warehouse Manager (PID: $WAREHOUSE_MANAGER_PID)..."
        kill $WAREHOUSE_MANAGER_PID 2>/dev/null || true
        wait $WAREHOUSE_MANAGER_PID 2>/dev/null || true
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
    echo -e "${BLUE} Building workspace...${NC}"
    cd "$WORKSPACE_DIR"
    colcon build --packages-select warehouse_robot_system
    
    if [ $? -ne 0 ]; then
        echo -e "${RED} Build failed!${NC}"
        exit 1
    fi
    
    echo -e "${GREEN} Build successful${NC}"
    
    # Start warehouse manager with default mode
    if ! start_warehouse_manager "none"; then
        echo -e "${RED} Failed to start warehouse manager${NC}"
        exit 1
    fi
    
    # Show initial menu
    show_menu
    
    # Interactive control loop
    while true; do
        echo -n -e "${CYAN}warehouse_manager>${NC} "
        read -r choice
        
        case $choice in
            1)
                switch_mode "inspection"
                ;;
            2)
                switch_mode "delivery"
                ;;
            3)
                switch_mode "manual"
                ;;
            s|S)
                start_operations
                ;;
            x|X)
                stop_operations
                ;;
            m|M)
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
                echo -e "Type ${PURPLE}'m'${NC} for menu or ${PURPLE}'q'${NC} to quit"
                ;;
        esac
    done
}

# Run main function
main "$@"