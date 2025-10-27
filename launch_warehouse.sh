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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
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
  std_msgs
  tf2
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

# Main Warehouse Robot System Executable (ONLY executable with main)
add_executable(warehouse_robot_system
  src/warehouse_robot_main.cpp
)
ament_target_dependencies(warehouse_robot_system
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(warehouse_robot_system
  warehouse_robot_lib
)

# ============================================================================
# INSTALL
# ============================================================================

# Install the single executable
install(TARGETS
  warehouse_robot_system
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
  warehouse_robot_lib
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