#!/bin/bash

# ============================================================================
# Prepare Camera Package for TurtleBot
# Creates a minimal package with only camera detection code
# ============================================================================

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Preparing Camera Package for TurtleBot              ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════╝${NC}"
echo ""

# Set paths
WORKSPACE_DIR="$HOME/MTRX3760_Project_2_Fixing/turtlebot3_ws"
SOURCE_DIR="$WORKSPACE_DIR/src/turtlebot3_simulations/warehouse_robot_system"
TEMP_DIR="/tmp/turtlebot_camera"

# Clean temp directory
echo " Cleaning temporary directory..."
rm -rf "$TEMP_DIR"
mkdir -p "$TEMP_DIR"

# Create package structure
echo " Creating package structure..."
mkdir -p "$TEMP_DIR/src/Camera"
mkdir -p "$TEMP_DIR/include/Camera"

# Copy camera source files
echo " Copying camera source files..."
cp "$SOURCE_DIR/src/Camera/AprilTagDetector.cpp" "$TEMP_DIR/src/Camera/"
cp "$SOURCE_DIR/src/Camera/ImageProcessor_Node.cpp" "$TEMP_DIR/src/Camera/"
cp "$SOURCE_DIR/src/Camera/ColourDetector.cpp" "$TEMP_DIR/src/Camera/"
cp "$SOURCE_DIR/src/Camera/Camera_Node.cpp" "$TEMP_DIR/src/Camera/" 2>/dev/null || echo "   ️  Camera_Node.cpp not found (optional)"

# Copy camera header files
echo " Copying camera header files..."
cp "$SOURCE_DIR/include/Camera/AprilTagDetector.hpp" "$TEMP_DIR/include/Camera/"
cp "$SOURCE_DIR/include/Camera/ImageProcessor_Node.hpp" "$TEMP_DIR/include/Camera/"
cp "$SOURCE_DIR/include/Camera/ColourDetector.hpp" "$TEMP_DIR/include/Camera/"
cp "$SOURCE_DIR/include/Camera/Camera_Node.hpp" "$TEMP_DIR/include/Camera/" 2>/dev/null || echo "   ️  Camera_Node.hpp not found (optional)"

# Create minimal package.xml
echo " Creating package.xml..."
cat > "$TEMP_DIR/package.xml" << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>turtlebot_camera</name>
  <version>1.0.0</version>
  <description>Camera detection package for TurtleBot - AprilTag and Color detection</description>
  <maintainer email="user@todo.todo">TurtleBot User</maintainer>
  <license>TODO</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>apriltag_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>apriltag</depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# Create minimal CMakeLists.txt
echo " Creating CMakeLists.txt..."
cat > "$TEMP_DIR/CMakeLists.txt" << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(turtlebot_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

# ImageProcessor library (shared by both nodes)
add_library(image_processor_lib
  src/Camera/ImageProcessor_Node.cpp
)

ament_target_dependencies(image_processor_lib
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(image_processor_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(image_processor_lib
  ${OpenCV_LIBS}
)

# AprilTag Detector Node
add_executable(apriltag_detector_node
  src/Camera/AprilTagDetector.cpp
)

ament_target_dependencies(apriltag_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  apriltag_msgs
  cv_bridge
)

target_include_directories(apriltag_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${apriltag_INCLUDE_DIRS}
)

target_link_libraries(apriltag_detector_node
  image_processor_lib
  ${OpenCV_LIBS}
  apriltag
)

# Color Detector Node
add_executable(colour_detector_node
  src/Camera/ColourDetector.cpp
)

ament_target_dependencies(colour_detector_node
  rclcpp
  sensor_msgs
  apriltag_msgs
  cv_bridge
)

target_include_directories(colour_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(colour_detector_node
  image_processor_lib
  ${OpenCV_LIBS}
)

# Install executables
install(TARGETS
  apriltag_detector_node
  colour_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
EOF

echo ""
echo -e "${GREEN} Package prepared successfully!${NC}"
echo ""
echo " Package location: $TEMP_DIR"
echo ""
echo " Package contents:"
ls -lh "$TEMP_DIR"
echo ""
echo " Source files:"
ls -lh "$TEMP_DIR/src/Camera/"
echo ""
echo " Header files:"
ls -lh "$TEMP_DIR/include/Camera/"
echo ""

# Create transfer script
echo " Creating transfer script..."
cat > "$TEMP_DIR/transfer_to_turtlebot.sh" << 'TRANSFER_EOF'
#!/bin/bash

# Transfer script - Run this on your laptop

if [ -z "$1" ]; then
    echo "Usage: $0 <turtlebot_ip>"
    echo "Example: $0 192.168.0.100"
    exit 1
fi

TURTLEBOT_IP=$1

echo " Transferring camera package to TurtleBot at $TURTLEBOT_IP..."
echo ""

# Transfer the package
scp -r /tmp/turtlebot_camera ubuntu@$TURTLEBOT_IP:~/camera_ws/src/

if [ $? -eq 0 ]; then
    echo ""
    echo " Transfer complete!"
    echo ""
    echo "Next steps:"
    echo "1. SSH into TurtleBot: ssh ubuntu@$TURTLEBOT_IP"
    echo "2. Build package: cd ~/camera_ws && colcon build"
    echo "3. Source workspace: source install/setup.bash"
    echo "4. Run detector: ros2 run turtlebot_camera apriltag_detector_node --ros-args -p show_visualization:=false"
else
    echo ""
    echo " Transfer failed!"
    echo "Check:"
    echo "  • TurtleBot IP is correct"
    echo "  • TurtleBot is powered on and connected"
    echo "  • SSH access is working: ssh ubuntu@$TURTLEBOT_IP"
fi
TRANSFER_EOF

chmod +x "$TEMP_DIR/transfer_to_turtlebot.sh"

echo -e "${YELLOW}═══════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}  Next Steps${NC}"
echo -e "${YELLOW}═══════════════════════════════════════════════════════${NC}"
echo ""
echo "1️⃣  Transfer to TurtleBot:"
echo "   /tmp/turtlebot_camera/transfer_to_turtlebot.sh <TURTLEBOT_IP>"
echo ""
echo "2️⃣  SSH into TurtleBot and build:"
echo "   ssh ubuntu@<TURTLEBOT_IP>"
echo "   cd ~/camera_ws"
echo "   colcon build --packages-select turtlebot_camera"
echo "   source install/setup.bash"
echo ""
echo "3️⃣  Test on TurtleBot:"
echo "   ros2 run turtlebot_camera apriltag_detector_node --ros-args -p show_visualization:=false"
echo ""
echo -e "${GREEN}Package is ready to transfer!${NC}"
