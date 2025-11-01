cmake_minimum_required(VERSION 3.8)
project(warehouse_robot_system)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(apriltag REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Generate action interfaces (must be before executables)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ReturnHome.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# ============================================================================
# SLAM UTILITY LIBRARIES
# ============================================================================

# Priority Queue library
add_library(priority_queue
  src/priority_queue.cpp
)
ament_target_dependencies(priority_queue
  rclcpp
  geometry_msgs
  nav_msgs
)

# PathPlanner library
add_library(path_planner_lib
  src/PathPlanner.cpp
)
ament_target_dependencies(path_planner_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
)
target_link_libraries(path_planner_lib
  priority_queue
  ${OpenCV_LIBS}
)

# Frontier Search library
add_library(FrontierSearch
  src/FrontierSearch.cpp
)
ament_target_dependencies(FrontierSearch
  rclcpp
  nav_msgs
  geometry_msgs
)
target_link_libraries(FrontierSearch
  path_planner_lib
)

# ============================================================================
# SLAM COMPONENT LIBRARIES (No main functions)
# ============================================================================

# SlamController Library
add_library(slam_controller_lib
  src/SlamController.cpp
)
ament_target_dependencies(slam_controller_lib
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ExplorationPlanner Library
add_library(exploration_planner_lib
  src/ExplorationPlanner.cpp
)
ament_target_dependencies(exploration_planner_lib
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)
target_link_libraries(exploration_planner_lib
  FrontierSearch
  path_planner_lib
)

# MotionController Library
add_library(motion_controller_lib
  src/MotionController.cpp
)
ament_target_dependencies(motion_controller_lib
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(motion_controller_lib
  path_planner_lib
)

# AutonomousExplorationRobot Library
add_library(autonomous_exploration_lib
  src/AutonomousExplorationRobot.cpp
)
ament_target_dependencies(autonomous_exploration_lib
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)
target_link_libraries(autonomous_exploration_lib
  slam_controller_lib
  exploration_planner_lib
  motion_controller_lib
)

# ============================================================================
# WAREHOUSE ROBOT SYSTEM
# ============================================================================

# Warehouse Robot System Library
add_library(warehouse_robot_lib
  src/warehouse_robot_system.cpp
)
ament_target_dependencies(warehouse_robot_lib
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
)
# Link all component libraries to the main system library
target_link_libraries(warehouse_robot_lib
  slam_controller_lib
  exploration_planner_lib
  motion_controller_lib
  path_planner_lib
)

# Main Warehouse Robot System Executable
add_executable(warehouse_robot_node
  src/warehouse_robot_main.cpp
)
ament_target_dependencies(warehouse_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(warehouse_robot_node
  warehouse_robot_lib
)

# Autonomous SLAM Exploration Executable
add_executable(autonomous_slam_node
  src/autonomous_slam_node.cpp
)
ament_target_dependencies(autonomous_slam_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(autonomous_slam_node
  autonomous_exploration_lib
)

# Delivery Robot Library
add_library(delivery_robot_lib
  src/DeliveryRobot.cpp
)
ament_target_dependencies(delivery_robot_lib
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
)
target_link_libraries(delivery_robot_lib
  slam_controller_lib
  motion_controller_lib
  path_planner_lib
  yaml-cpp
)

# Delivery Robot Node
add_executable(delivery_robot_node
  src/delivery_robot_node.cpp
)
ament_target_dependencies(delivery_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
)
target_link_libraries(delivery_robot_node
  delivery_robot_lib
)

# Twist to TwistStamped Bridge
add_executable(twist_stamped_bridge
  src/twist_stamped_bridge.cpp
)
ament_target_dependencies(twist_stamped_bridge
  rclcpp
  geometry_msgs
)

# Zone Marker Node (for interactive zone definition)
add_executable(zone_marker_node
  src/zone_marker_node.cpp
)
ament_target_dependencies(zone_marker_node
  rclcpp
  geometry_msgs
  visualization_msgs
  std_srvs
)
target_link_libraries(zone_marker_node
  yaml-cpp
)

# Add executable for colour detector
add_executable(colour_detector_node 
  src/ColourDetector.cpp
  src/ImageProcessor_Node.cpp
)

# Link dependencies
ament_target_dependencies(colour_detector_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV 
  apriltag_msgs 
  std_msgs
)

# Add executable for independent AprilTag detector (main() is now in AprilTagDetector.cpp)
add_executable(apriltag_detector_node 
  src/AprilTagDetector.cpp
  src/ImageProcessor_Node.cpp
)

# Link dependencies
ament_target_dependencies(apriltag_detector_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV 
  apriltag_msgs 
  std_msgs
)

# Link native apriltag library
target_link_libraries(apriltag_detector_node 
  apriltag::apriltag
)

# ============================================================================
# INSTALL
# ============================================================================

# Install executables
install(TARGETS
  warehouse_robot_node
  autonomous_slam_node
  delivery_robot_node
  twist_stamped_bridge
  zone_marker_node
  colour_detector_node
  apriltag_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install all libraries
install(TARGETS
  priority_queue
  path_planner_lib
  FrontierSearch
  slam_controller_lib
  exploration_planner_lib
  motion_controller_lib
  autonomous_exploration_lib
  warehouse_robot_lib
  delivery_robot_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files (if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch/
  )
endif()

# Install config files (if directory exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()