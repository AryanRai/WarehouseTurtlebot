#!/bin/bash
# Script to check SLAM controller logs for common issues

echo "=== SLAM Controller Log Analyzer ==="
echo ""
echo "Checking for common issues in recent logs..."
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if ros2 is running
if ! pgrep -x "ros2" > /dev/null; then
    echo -e "${YELLOW}Warning: ROS2 doesn't appear to be running${NC}"
    echo ""
fi

# Function to check for patterns in logs
check_pattern() {
    local pattern=$1
    local description=$2
    local log_file="/tmp/slam_logs.txt"
    
    # Capture recent logs
    ros2 topic echo /rosout --once 2>/dev/null > $log_file
    
    if grep -q "$pattern" $log_file 2>/dev/null; then
        echo -e "${RED}✗${NC} Found: $description"
        return 1
    else
        echo -e "${GREEN}✓${NC} OK: $description"
        return 0
    fi
}

echo "Checking for common navigation issues:"
echo ""

# Check for circling behavior indicators
echo "1. Checking for stuck/circling indicators..."
check_pattern "Robot appears stuck" "No stuck warnings"
check_pattern "No paths found" "Path planning working"
check_pattern "No frontiers" "Frontiers being detected"
echo ""

# Check state transitions
echo "2. Checking state machine behavior..."
ros2 topic echo /rosout --once 2>/dev/null | grep -i "state transition" | tail -5
echo ""

# Check velocity commands
echo "3. Checking velocity commands..."
echo "Recent linear velocities:"
ros2 topic echo /cmd_vel --once 2>/dev/null | grep -A 1 "linear:" | grep "x:" | tail -5
echo ""
echo "Recent angular velocities:"
ros2 topic echo /cmd_vel --once 2>/dev/null | grep -A 1 "angular:" | grep "z:" | tail -5
echo ""

# Check if robot is moving
echo "4. Checking if robot is moving..."
ODOM1=$(ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | grep "x:" | awk '{print $2}')
sleep 2
ODOM2=$(ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | grep "x:" | awk '{print $2}')

if [ ! -z "$ODOM1" ] && [ ! -z "$ODOM2" ]; then
    DIFF=$(echo "$ODOM2 - $ODOM1" | bc 2>/dev/null)
    if [ ! -z "$DIFF" ]; then
        ABS_DIFF=$(echo "$DIFF" | tr -d '-')
        if (( $(echo "$ABS_DIFF > 0.01" | bc -l) )); then
            echo -e "${GREEN}✓${NC} Robot is moving (Δx = $DIFF m)"
        else
            echo -e "${YELLOW}!${NC} Robot appears stationary (Δx = $DIFF m)"
        fi
    fi
else
    echo -e "${YELLOW}!${NC} Could not determine robot movement"
fi
echo ""

echo "=== Analysis Complete ==="
echo ""
echo "Tips:"
echo "  - If robot is stuck, check for obstacles in simulation"
echo "  - If no frontiers found, exploration may be complete"
echo "  - If circling, check the path visualization in RViz"
echo "  - Use 'ros2 topic echo /slam/planned_path' to see current path"
echo ""
