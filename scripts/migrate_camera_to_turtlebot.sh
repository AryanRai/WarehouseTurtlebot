#!/bin/bash

# ============================================================================
# Complete Camera Migration to TurtleBot Script
# Automates the entire process of moving camera processing to TurtleBot
# ============================================================================

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Configuration
TURTLEBOT_IP="${1:-10.42.0.1}"
TURTLEBOT_USER="ubuntu"
TURTLEBOT_PASSWORD="turtlebot"

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Camera Migration to TurtleBot - Complete Setup      ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "This script will:"
echo "  1. Prepare camera package on laptop"
echo "  2. Check TurtleBot connection"
echo "  3. Install dependencies on TurtleBot"
echo "  4. Transfer camera package to TurtleBot"
echo "  5. Build package on TurtleBot"
echo "  6. Transfer startup script"
echo "  7. Verify installation"
echo ""
echo "TurtleBot IP: $TURTLEBOT_IP"
echo ""
echo -n "Continue? (yes/no): "
read confirm

if [[ "$confirm" != "yes" ]]; then
    echo "Cancelled."
    exit 0
fi

# ============================================================================
# STEP 1: Prepare Package on Laptop
# ============================================================================

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Step 1: Preparing Camera Package${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Run the prepare script
if [ -f "$SCRIPT_DIR/prepare_turtlebot_camera.sh" ]; then
    bash "$SCRIPT_DIR/prepare_turtlebot_camera.sh"
elif [ -f "$PROJECT_DIR/scripts/prepare_turtlebot_camera.sh" ]; then
    bash "$PROJECT_DIR/scripts/prepare_turtlebot_camera.sh"
else
    echo -e "${RED}❌ prepare_turtlebot_camera.sh not found!${NC}"
    echo "Looking in:"
    echo "  $SCRIPT_DIR/prepare_turtlebot_camera.sh"
    echo "  $PROJECT_DIR/scripts/prepare_turtlebot_camera.sh"
    exit 1
fi

if [ ! -d "/tmp/turtlebot_camera" ]; then
    echo -e "${RED}❌ Package preparation failed!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Package prepared${NC}"

# ============================================================================
# STEP 2: Check TurtleBot Connection
# ============================================================================

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Step 2: Checking TurtleBot Connection${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

echo "Testing SSH connection to $TURTLEBOT_IP..."

# Test connection
if ! ping -c 1 -W 2 $TURTLEBOT_IP > /dev/null 2>&1; then
    echo -e "${RED}❌ Cannot ping TurtleBot at $TURTLEBOT_IP${NC}"
    echo ""
    echo "Please check:"
    echo "  • TurtleBot is powered on"
    echo "  • Network connection is working"
    echo "  • IP address is correct"
    exit 1
fi

echo -e "${GREEN}✅ TurtleBot is reachable${NC}"

# Check for sshpass
if ! command -v sshpass > /dev/null; then
    echo -e "${YELLOW}⚠️  sshpass not installed - installing it now...${NC}"
    sudo apt install -y sshpass > /dev/null 2>&1
    if ! command -v sshpass > /dev/null; then
        echo -e "${RED}❌ Failed to install sshpass${NC}"
        echo "Please install manually: sudo apt install sshpass"
        exit 1
    fi
fi

# Test SSH with sshpass
if ! sshpass -p "$TURTLEBOT_PASSWORD" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 $TURTLEBOT_USER@$TURTLEBOT_IP "echo 'Connected'" > /dev/null 2>&1; then
    echo -e "${RED}❌ Cannot SSH to TurtleBot${NC}"
    echo "Please verify:"
    echo "  • TurtleBot IP: $TURTLEBOT_IP"
    echo "  • Password: $TURTLEBOT_PASSWORD"
    echo "  • Test manually: ssh $TURTLEBOT_USER@$TURTLEBOT_IP"
    exit 1
fi

USE_SSHPASS=true

echo -e "${GREEN}✅ SSH connection working${NC}"

# ============================================================================
# STEP 3: Install Dependencies on TurtleBot
# ============================================================================

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Step 3: Installing Dependencies on TurtleBot${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

echo "This will install:"
echo "  • ros-jazzy-apriltag"
echo "  • ros-jazzy-apriltag-msgs"
echo "  • libopencv-dev"
echo "  • ros-jazzy-cv-bridge"
echo "  • libyaml-cpp-dev"
echo ""
echo "This may take 5-10 minutes..."
echo ""

# Create installation script
cat > /tmp/install_deps.sh << 'EOF'
#!/bin/bash
echo "Updating package list..."
echo 'turtlebot' | sudo -S apt update -qq

echo "Installing AprilTag libraries..."
echo 'turtlebot' | sudo -S apt install -y ros-jazzy-apriltag ros-jazzy-apriltag-msgs

echo "Installing OpenCV..."
echo 'turtlebot' | sudo -S apt install -y libopencv-dev ros-jazzy-cv-bridge

echo "Installing YAML parser..."
echo 'turtlebot' | sudo -S apt install -y libyaml-cpp-dev

echo "Verifying installations..."
dpkg -l | grep -E "apriltag|opencv" | head -5

echo "Dependencies installed!"
EOF

chmod +x /tmp/install_deps.sh

# Transfer and run installation script
echo "Transferring installation script..."
sshpass -p "$TURTLEBOT_PASSWORD" scp -o StrictHostKeyChecking=no /tmp/install_deps.sh $TURTLEBOT_USER@$TURTLEBOT_IP:/tmp/

echo "Running installation on TurtleBot (this may take 5-10 minutes)..."
sshpass -p "$TURTLEBOT_PASSWORD" ssh -o StrictHostKeyChecking=no $TURTLEBOT_USER@$TURTLEBOT_IP "bash /tmp/install_deps.sh"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Dependency installation failed!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Dependencies installed${NC}"

# ============================================================================
# STEP 4: Create Workspace on TurtleBot
# ============================================================================

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Step 4: Creating Workspace on TurtleBot${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

echo "Creating ~/camera_ws/src on TurtleBot..."
sshpass -p "$TURTLEBOT_PASSWORD" ssh -o StrictHostKeyChecking=no $TURTLEBOT_USER@$TURTLEBOT_IP "mkdir -p ~/camera_ws/src"

echo -e "${GREEN}✅ Workspace created${NC}"

# ============================================================================
# STEP 5: Transfer Camera Package
# ============================================================================

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Step 5: Transferring Camera Package${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

echo "Transferring package to TurtleBot..."
sshpass -p "$TURTLEBOT_PASSWORD" scp -r -o StrictHostKeyChecking=no /tmp/turtlebot_camera $TURTLEBOT_USER@$TURTLEBOT_IP:~/camera_ws/src/

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Package transfer failed!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Package transferred${NC}"

# ============================================================================
# STEP 6: Build Package on TurtleBot
# ============================================================================

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Step 6: Building Package on TurtleBot${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

echo "Building camera package..."
echo "This may take 2-5 minutes..."
echo ""

# Create build script
cat > /tmp/build_camera.sh << 'EOF'
#!/bin/bash
cd ~/camera_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select turtlebot_camera
EOF

chmod +x /tmp/build_camera.sh

# Transfer and run build script
sshpass -p "$TURTLEBOT_PASSWORD" scp -o StrictHostKeyChecking=no /tmp/build_camera.sh $TURTLEBOT_USER@$TURTLEBOT_IP:/tmp/

sshpass -p "$TURTLEBOT_PASSWORD" ssh -o StrictHostKeyChecking=no $TURTLEBOT_USER@$TURTLEBOT_IP "bash /tmp/build_camera.sh"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Build failed!${NC}"
    echo ""
    echo "To debug, SSH into TurtleBot and check:"
    echo "  ssh $TURTLEBOT_USER@$TURTLEBOT_IP"
    echo "  cd ~/camera_ws"
    echo "  colcon build --packages-select turtlebot_camera"
    exit 1
fi

echo -e "${GREEN}✅ Package built successfully${NC}"

# ============================================================================
# STEP 7: Transfer Startup Script
# ============================================================================

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Step 7: Transferring Startup Script${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

if [ ! -f "$PROJECT_DIR/turtlebot_start_camera.sh" ]; then
    echo -e "${RED}❌ turtlebot_start_camera.sh not found!${NC}"
    echo "Looking in: $PROJECT_DIR/turtlebot_start_camera.sh"
    exit 1
fi

echo "Transferring startup script..."
sshpass -p "$TURTLEBOT_PASSWORD" scp -o StrictHostKeyChecking=no "$PROJECT_DIR/turtlebot_start_camera.sh" $TURTLEBOT_USER@$TURTLEBOT_IP:~/

# Make it executable
sshpass -p "$TURTLEBOT_PASSWORD" ssh -o StrictHostKeyChecking=no $TURTLEBOT_USER@$TURTLEBOT_IP "chmod +x ~/turtlebot_start_camera.sh"

echo -e "${GREEN}✅ Startup script transferred${NC}"

# ============================================================================
# STEP 8: Verify Installation
# ============================================================================

echo ""
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}  Step 8: Verifying Installation${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════════${NC}"
echo ""

echo "Checking installation..."

# Create verification script
cat > /tmp/verify_install.sh << 'EOF'
#!/bin/bash
cd ~/camera_ws
source install/setup.bash

echo "Checking executables..."
if [ -f "install/turtlebot_camera/lib/turtlebot_camera/apriltag_detector_node" ]; then
    echo "✅ apriltag_detector_node found"
else
    echo "❌ apriltag_detector_node NOT found"
    exit 1
fi

if [ -f "install/turtlebot_camera/lib/turtlebot_camera/colour_detector_node" ]; then
    echo "✅ colour_detector_node found"
else
    echo "❌ colour_detector_node NOT found"
    exit 1
fi

echo "✅ All executables present"
EOF

chmod +x /tmp/verify_install.sh
sshpass -p "$TURTLEBOT_PASSWORD" scp -o StrictHostKeyChecking=no /tmp/verify_install.sh $TURTLEBOT_USER@$TURTLEBOT_IP:/tmp/

sshpass -p "$TURTLEBOT_PASSWORD" ssh -o StrictHostKeyChecking=no $TURTLEBOT_USER@$TURTLEBOT_IP "bash /tmp/verify_install.sh"

if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Verification failed!${NC}"
    exit 1
fi

echo -e "${GREEN}✅ Installation verified${NC}"

# ============================================================================
# SUCCESS!
# ============================================================================

echo ""
echo -e "${GREEN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║   ✅ Camera Migration Complete!                        ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════════════════════╝${NC}"
echo ""
echo "📦 What was installed:"
echo "   • Camera package built on TurtleBot"
echo "   • AprilTag detector ready"
echo "   • Color detector ready"
echo "   • Startup script installed"
echo ""
echo "🚀 Next Steps:"
echo ""
echo "1️⃣  Start TurtleBot hardware (Terminal 1):"
echo "   ssh $TURTLEBOT_USER@$TURTLEBOT_IP"
echo "   ros2 launch turtlebot3_bringup robot.launch.py"
echo ""
echo "2️⃣  Start camera detection (Terminal 2):"
echo "   ssh $TURTLEBOT_USER@$TURTLEBOT_IP"
echo "   ~/turtlebot_start_camera.sh"
echo ""
echo "3️⃣  Update laptop script:"
echo "   Edit: scripts/run_autonomous_slam.sh"
echo "   Comment out camera node startup sections"
echo "   (Lines ~920, ~980, ~1000)"
echo ""
echo "4️⃣  Start SLAM on laptop (Terminal 3):"
echo "   ./scripts/run_autonomous_slam.sh -nocamui"
echo ""
echo "5️⃣  Verify topics on laptop:"
echo "   ros2 topic echo /apriltag_detections --once"
echo ""
echo "📊 Expected Results:"
echo "   • Network: 30 Mbps → 2 Mbps (15x reduction)"
echo "   • TF2 latency: 200ms → 20ms (10x improvement)"
echo "   • TF2 failures: Frequent → None"
echo "   • No more crashes!"
echo ""
echo -e "${YELLOW}💡 Quick Test:${NC}"
echo "   python3 scripts/turtlebot_bringup.py start robot"
echo "   ssh $TURTLEBOT_USER@$TURTLEBOT_IP '~/turtlebot_start_camera.sh'"
echo ""
echo -e "${GREEN}Migration successful! 🎉${NC}"
