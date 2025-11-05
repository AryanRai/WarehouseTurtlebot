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
find_package(apriltag_msgs QUIET)

# Try to find nlohmann_json
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(NLOHMANN_JSON nlohmann_json)
    if(NLOHMANN_JSON_FOUND)
      message(STATUS "Found nlohmann_json via pkg-config")
    endif()
  endif()
endif()

# If still not found, check for header file
if(NOT nlohmann_json_FOUND AND NOT NLOHMANN_JSON_FOUND)
  find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
    PATHS /usr/include /usr/local/include
  )
  if(NLOHMANN_JSON_INCLUDE_DIR)
    message(STATUS "Found nlohmann/json.hpp at ${NLOHMANN_JSON_INCLUDE_DIR}")
    set(NLOHMANN_JSON_FOUND TRUE)
  else()
    message(WARNING "nlohmann_json not found - install with: sudo apt install nlohmann-json3-dev")
  endif()
endif()

# Try to find apriltag library (optional for apriltag_detector_node)
find_package(apriltag QUIET)
if(apriltag_FOUND)
  message(STATUS "Found apriltag library - will build apriltag_detector_node")
  set(BUILD_APRILTAG_DETECTOR TRUE)
else()
  message(WARNING "apriltag library not found - skipping apriltag_detector_node")
  message(WARNING "Install with: sudo apt install ros-${ROS_DISTRO}-apriltag")
  set(BUILD_APRILTAG_DETECTOR FALSE)
endif()

# Include directories - ADD SUBDIRECTORIES
include_directories(
  include
  include/Camera
  include/Robot
  include/SLAM
  include/Visualization
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Add nlohmann_json include if found
if(NLOHMANN_JSON_INCLUDE_DIR)
  include_directories(${NLOHMANN_JSON_INCLUDE_DIR})
elseif(NLOHMANN_JSON_INCLUDE_DIRS)
  include_directories(${NLOHMANN_JSON_INCLUDE_DIRS})
endif()

# Generate action interfaces (must be before executables)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ReturnHome.action"
  DEPENDENCIES geometry_msgs std_msgs
)

# ============================================================================
# SLAM UTILITY LIBRARIES
# ============================================================================

# PathPlanner library - UPDATED PATH
add_library(path_planner_lib
  src/SLAM/PathPlanner.cpp
)
ament_target_dependencies(path_planner_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_geometry_msgs
)
target_link_libraries(path_planner_lib
  ${OpenCV_LIBS}
)

# Frontier Search library - UPDATED PATH
add_library(FrontierSearch
  src/SLAM/FrontierSearch.cpp
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
# SLAM COMPONENT LIBRARIES
# ============================================================================

# SlamController Library - UPDATED PATH
add_library(slam_controller_lib
  src/SLAM/SlamController.cpp
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

# ExplorationPlanner Library - UPDATED PATH
add_library(exploration_planner_lib
  src/SLAM/ExplorationPlanner.cpp
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

# MotionController Library - UPDATED PATH
add_library(motion_controller_lib
  src/SLAM/MotionController.cpp
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

# AutonomousExplorationRobot Library - UPDATED PATH
add_library(autonomous_exploration_lib
  src/Robot/AutonomousExplorationRobot.cpp
)
ament_target_dependencies(autonomous_exploration_lib
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
)
target_link_libraries(autonomous_exploration_lib
  slam_controller_lib
  exploration_planner_lib
  motion_controller_lib
  yaml-cpp
)

# Link nlohmann_json if available
if(nlohmann_json_FOUND)
  target_link_libraries(autonomous_exploration_lib nlohmann_json::nlohmann_json)
elseif(NLOHMANN_JSON_LIBRARIES)
  target_link_libraries(autonomous_exploration_lib ${NLOHMANN_JSON_LIBRARIES})
endif()

# ============================================================================
# ROBOT EXECUTABLES
# ============================================================================

# Autonomous SLAM Exploration Executable - UPDATED PATH
add_executable(autonomous_slam_node
  src/Robot/autonomous_slam_node.cpp
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
  apriltag_msgs
)
target_link_libraries(autonomous_slam_node
  autonomous_exploration_lib
)

# ============================================================================
# ROBOT BASE CLASS AND MANAGER LIBRARIES
# ============================================================================

# Warehouse Robot Base Class Library
add_library(warehouse_robot_base_lib
  src/Robot/WarehouseRobot.cpp
)
ament_target_dependencies(warehouse_robot_base_lib
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(warehouse_robot_base_lib
  slam_controller_lib
  motion_controller_lib
  path_planner_lib
)

# Warehouse Manager Library (Polymorphic Robot Control) - COMMENTED OUT due to missing dependencies
# add_library(warehouse_manager_lib
#   src/Robot/WarehouseManager.cpp
# )
# ament_target_dependencies(warehouse_manager_lib
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   std_msgs
# )
# target_link_libraries(warehouse_manager_lib
#   warehouse_robot_base_lib
# )

# ============================================================================
# DERIVED ROBOT LIBRARIES
# ============================================================================

# Delivery Robot Library - UPDATED PATH
add_library(delivery_robot_lib
  src/Robot/DeliveryRobot.cpp
)
ament_target_dependencies(delivery_robot_lib
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
)
target_link_libraries(delivery_robot_lib
  warehouse_robot_base_lib
  slam_controller_lib
  motion_controller_lib
  path_planner_lib
  yaml-cpp
)

# Delivery Robot Node - UPDATED PATH
add_executable(delivery_robot_node
  src/Robot/delivery_robot_node.cpp
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

# Inspection Robot Library - UPDATED PATH
add_library(inspection_robot_lib
  src/Robot/InspectionRobot.cpp
)
ament_target_dependencies(inspection_robot_lib
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

# Add apriltag_msgs dependency if available
if(apriltag_msgs_FOUND)
  ament_target_dependencies(inspection_robot_lib apriltag_msgs)
endif()
target_link_libraries(inspection_robot_lib
  warehouse_robot_base_lib
  slam_controller_lib
  motion_controller_lib
  path_planner_lib
  yaml-cpp
)

# Inspection Robot Node - UPDATED PATH
add_executable(inspection_robot_node
  src/Robot/inspection_robot_node.cpp
)
ament_target_dependencies(inspection_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

# Add apriltag_msgs dependency if available
if(apriltag_msgs_FOUND)
  ament_target_dependencies(inspection_robot_node apriltag_msgs)
endif()
target_link_libraries(inspection_robot_node
  inspection_robot_lib
)

# Dynamic Warehouse Manager Node - NEW
add_executable(warehouse_manager_node
  src/Robot/warehouse_manager_node.cpp
)
ament_target_dependencies(warehouse_manager_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

# Add apriltag_msgs dependency if available
if(apriltag_msgs_FOUND)
  ament_target_dependencies(warehouse_manager_node apriltag_msgs)
endif()
target_link_libraries(warehouse_manager_node
  delivery_robot_lib
  inspection_robot_lib
  camera_lib
)

# Twist to TwistStamped Bridge - UPDATED PATH
add_executable(twist_stamped_bridge
  src/SLAM/twist_stamped_bridge.cpp
)
ament_target_dependencies(twist_stamped_bridge
  rclcpp
  geometry_msgs
)

# Zone Marker Node - UPDATED PATH
add_executable(zone_marker_node
  src/SLAM/zone_marker_node.cpp
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

# ============================================================================
# CAMERA NODES
# ============================================================================

# Camera Node - UPDATED PATH
add_executable(camera_node
  src/Camera/Camera_Node.cpp
)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
)

# Colour Detector Node - UPDATED PATHS
add_executable(colour_detector_node 
  src/Camera/ColourDetector.cpp
  src/Camera/ImageProcessor_Node.cpp
)
ament_target_dependencies(colour_detector_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  OpenCV 
  apriltag_msgs 
  std_msgs
)

# AprilTag Detector Node (conditional) - UPDATED PATHS
if(BUILD_APRILTAG_DETECTOR)
  add_executable(apriltag_detector_node 
    src/Camera/AprilTagDetector.cpp
    src/Camera/ImageProcessor_Node.cpp
  )

  ament_target_dependencies(apriltag_detector_node 
    rclcpp 
    sensor_msgs 
    cv_bridge 
    OpenCV 
    apriltag_msgs 
    std_msgs
  )

  target_link_libraries(apriltag_detector_node 
    apriltag::apriltag
  )
endif()

# Exploration Path Tracker Node (Visualization) - COMMENTED OUT - Missing files
# add_executable(exploration_path_tracker_node
#   src/Visualization/exploration_path_tracker_node.cpp
#   src/Visualization/ExplorationPathTracker.cpp
# )
# 
# ament_target_dependencies(exploration_path_tracker_node
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   visualization_msgs
#   std_msgs
# )

# ============================================================================
# INSTALL
# ============================================================================

# Install executables
set(INSTALL_TARGETS
  autonomous_slam_node
  delivery_robot_node
  inspection_robot_node
  warehouse_manager_node
  twist_stamped_bridge
  zone_marker_node
  camera_node
  colour_detector_node
  # exploration_path_tracker_node  # Commented out - missing files
)

# Add apriltag_detector_node if it was built
if(BUILD_APRILTAG_DETECTOR)
  list(APPEND INSTALL_TARGETS apriltag_detector_node)
endif()

install(TARGETS
  ${INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install all libraries
install(TARGETS
  path_planner_lib
  FrontierSearch
  slam_controller_lib
  exploration_planner_lib
  motion_controller_lib
  autonomous_exploration_lib
  delivery_robot_lib
  inspection_robot_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files - UPDATED to include subdirectories
install(DIRECTORY 
  include/Camera
  include/Robot
  include/SLAM
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