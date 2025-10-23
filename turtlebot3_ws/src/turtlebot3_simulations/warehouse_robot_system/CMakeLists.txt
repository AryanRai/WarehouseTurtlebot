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

# OpenCV for image processing and SLAM
find_package(OpenCV REQUIRED)

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
)

target_link_libraries(slam_module
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

# -------------------- Executables --------------------
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

# -------------------- Install --------------------
install(TARGETS
  slam_module
  ${PROJECT_NAME}_lib
  warehouse_robot_main
  slam_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
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
  OpenCV
)

ament_package()