cmake_minimum_required(VERSION 3.8)
project(warehouse_robot_system)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------- Dependencies --------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# OpenCV for image processing and SLAM
find_package(OpenCV REQUIRED)

# Camera detection dependencies
find_package(cv_bridge REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(image_transport REQUIRED)

# -------------------- Include directories --------------------
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# -------------------- SLAM Module Library --------------------
add_library(slam_module
  src/path_planner.cpp
  src/frontier_search.cpp
  src/priority_queue.cpp
  src/exploration_manager.cpp
  src/navigation_controller.cpp
  src/slam_state_manager.cpp
)

target_include_directories(slam_module PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(slam_module
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(slam_module
  ${OpenCV_LIBS}
)

# -------------------- Autonomous SLAM Library --------------------
add_library(autonomous_slam_lib
  src/autonomous_slam_controller.cpp
)

target_include_directories(autonomous_slam_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(autonomous_slam_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(autonomous_slam_lib
  slam_module
  ${OpenCV_LIBS}
)

# -------------------- Warehouse Robot System Library --------------------
add_library(${PROJECT_NAME}_lib
  src/warehouse_robot_system.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}_lib
  slam_module
  ${OpenCV_LIBS}
)

# -------------------- Camera Detection Module --------------------
# Image Processor Base Library
add_library(image_processor_base
  src/camera_detection/CImageProcessorNode.cpp
)

target_include_directories(image_processor_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(image_processor_base
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Camera Node (runs on TurtleBot)
add_executable(camera_node 
  src/camera_detection/CCameraNode.cpp
)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
)

# AprilTag Detector Node (runs on laptop)
add_executable(apriltag_detector_node
  src/camera_detection/CAprilTagDetector.cpp
)

target_link_libraries(apriltag_detector_node
  image_processor_base
)

ament_target_dependencies(apriltag_detector_node
  rclcpp
  sensor_msgs
  apriltag_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

# Colour Detector Node (runs on laptop)
add_executable(colour_detector_node
  src/camera_detection/CColourDetector.cpp
)

target_link_libraries(colour_detector_node
  image_processor_base
)

ament_target_dependencies(colour_detector_node
  rclcpp
  sensor_msgs
  apriltag_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  OpenCV
)

# -------------------- Existing Executables --------------------
# Main warehouse robot system executable
add_executable(warehouse_robot_main
  src/warehouse_robot_main.cpp
)

target_link_libraries(warehouse_robot_main
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(warehouse_robot_main
  rclcpp
)

# SLAM module test executable
add_executable(slam_test
  src/slam_test.cpp
)

target_link_libraries(slam_test
  slam_module
)

ament_target_dependencies(slam_test
  rclcpp
  nav_msgs
)

# Autonomous SLAM executable
add_executable(autonomous_slam_main
  src/autonomous_slam_main.cpp
)

target_link_libraries(autonomous_slam_main
  autonomous_slam_lib
)

ament_target_dependencies(autonomous_slam_main
  rclcpp
)

# -------------------- Install --------------------
install(TARGETS
  slam_module
  autonomous_slam_lib
  ${PROJECT_NAME}_lib
  image_processor_base
  warehouse_robot_main
  slam_test
  autonomous_slam_main
  camera_node
  apriltag_detector_node
  colour_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# -------------------- Ament export --------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  cv_bridge
  apriltag_msgs
  image_transport
)

ament_package()