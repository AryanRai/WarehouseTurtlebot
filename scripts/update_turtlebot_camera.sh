#!/bin/bash

# Quick script to update camera package on TurtleBot after fixing CMakeLists

TURTLEBOT_IP="${1:-10.42.0.1}"
TURTLEBOT_USER="ubuntu"
TURTLEBOT_PASSWORD="turtlebot"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}Updating Camera Package on TurtleBot${NC}"
echo "======================================"
echo ""

# Check sshpass
if ! command -v sshpass > /dev/null; then
    echo -e "${RED} sshpass not installed${NC}"
    echo "Install with: sudo apt install sshpass"
    exit 1
fi

# Prepare package
echo "1️⃣ Preparing package..."
bash scripts/prepare_turtlebot_camera.sh > /dev/null 2>&1

if [ ! -d "/tmp/turtlebot_camera" ]; then
    echo -e "${RED} Package preparation failed${NC}"
    exit 1
fi

echo -e "${GREEN} Package prepared${NC}"

# Remove old package on TurtleBot
echo ""
echo "2️⃣ Removing old package on TurtleBot..."
sshpass -p "$TURTLEBOT_PASSWORD" ssh -o StrictHostKeyChecking=no $TURTLEBOT_USER@$TURTLEBOT_IP "rm -rf ~/camera_ws/src/turtlebot_camera ~/camera_ws/build/turtlebot_camera ~/camera_ws/install/turtlebot_camera"

echo -e "${GREEN} Old package removed${NC}"

# Transfer new package
echo ""
echo "3️⃣ Transferring new package..."
sshpass -p "$TURTLEBOT_PASSWORD" scp -r -o StrictHostKeyChecking=no /tmp/turtlebot_camera $TURTLEBOT_USER@$TURTLEBOT_IP:~/camera_ws/src/ > /dev/null 2>&1

echo -e "${GREEN} Package transferred${NC}"

# Build
echo ""
echo "4️⃣ Building on TurtleBot..."
sshpass -p "$TURTLEBOT_PASSWORD" ssh -o StrictHostKeyChecking=no $TURTLEBOT_USER@$TURTLEBOT_IP "cd ~/camera_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select turtlebot_camera"

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║    Update Complete!                  ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
    echo ""
    echo "Test it:"
    echo "  sshpass -p turtlebot ssh ubuntu@$TURTLEBOT_IP '~/turtlebot_start_camera.sh'"
else
    echo ""
    echo -e "${RED} Build failed${NC}"
    echo "Check errors above"
    exit 1
fi
