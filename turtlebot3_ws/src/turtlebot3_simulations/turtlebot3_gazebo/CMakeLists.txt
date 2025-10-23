cmake_minimum_required(VERSION 3.8)
project(turtlebot3_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------- Dependencies --------------------
find_package(ament_cmake REQUIRED)

# ROS 2 / msgs
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Gazebo (gz) stack
find_package(ros_gz_sim REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

# Vision stack for camera red detector
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# -------------------- Common library --------------------
add_library(turtlebot3_gazebo_core
  src/CTurtlebot3Drive.cpp
  src/CTurtlebot3Sensor.cpp
  src/CWallFollowingController.cpp
  src/CPIDController.cpp
)

target_compile_features(turtlebot3_gazebo_core PUBLIC cxx_std_17)

target_include_directories(turtlebot3_gazebo_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(turtlebot3_gazebo_core
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
)

# Link gz imported targets
target_link_libraries(turtlebot3_gazebo_core
  gz-sim::gz-sim
  gz-math::gz-math
)

# -------------------- Executables --------------------
# Drive node
add_executable(CTurtlebot3Drive_node src/CTurtlebot3Drive.cpp)
target_link_libraries(CTurtlebot3Drive_node
  turtlebot3_gazebo_core
)
ament_target_dependencies(CTurtlebot3Drive_node rclcpp)

# Camera red detector
add_executable(camera_red_detector src/CCameraRedDetector.cpp)
target_include_directories(camera_red_detector PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(camera_red_detector
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
)
target_link_libraries(camera_red_detector
  ${OpenCV_LIBS}
)

# Obstacles — Gazebo plugins
set(OBSTACLE_LIBS obstacles obstacle1 obstacle2)
foreach(lib ${OBSTACLE_LIBS})
  add_library(${lib} SHARED src/${lib}.cpp)
  target_include_directories(${lib} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(${lib}
    gz-sim::gz-sim
    gz-math::gz-math
  )
endforeach()

# -------------------- Install --------------------
install(TARGETS
  turtlebot3_gazebo_core
  CTurtlebot3Drive_node
  camera_red_detector
  obstacles
  obstacle1
  obstacle2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models params rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

# -------------------- Ament export --------------------
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  image_transport
  cv_bridge
  OpenCV
)
ament_package()