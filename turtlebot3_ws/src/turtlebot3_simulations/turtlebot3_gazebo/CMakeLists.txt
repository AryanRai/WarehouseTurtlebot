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

# Link gz imported targets (plain signature)
target_link_libraries(turtlebot3_gazebo_core
  gz-sim::gz-sim
  gz-math::gz-math
)

# -------------------- Executables --------------------
# Drive node (thin wrapper that links the core lib)
# NOTE: If you don't have src/drive_node_main.cpp, either create it with a main()
# or change this to build your existing file that contains main().
add_executable(CTurtlebot3Drive_node src/CTurtlebot3Drive.cpp)
target_link_libraries(CTurtlebot3Drive_node
  turtlebot3_gazebo_core
)
ament_target_dependencies(CTurtlebot3Drive_node rclcpp)

# Camera red detector — standalone executable
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
# Plain signature (no PRIVATE/PUBLIC keywords)
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
  # Plain signature here as well
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






























# cmake_minimum_required(VERSION 3.8)
# project(turtlebot3_gazebo)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 17)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# if(MSVC)
#   add_compile_definitions(_USE_MATH_DEFINES)
# endif()

# # -------------------- Dependencies --------------------
# find_package(ament_cmake REQUIRED)
# find_package(ros_gz_sim REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(gz_math_vendor REQUIRED)
# find_package(gz-math REQUIRED)
# find_package(gz_sim_vendor REQUIRED)
# find_package(gz-sim REQUIRED)
# find_package(gz_plugin_vendor REQUIRED)
# find_package(gz-plugin REQUIRED)

# # For red_detector
# find_package(std_msgs REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)

# # ----- IMPORTANT: DO NOT find rclcpp_components; we're not using components -----

# # -------------------- Build --------------------
# link_directories(
#   ${GAZEBO_LIBRARY_DIRS}
#   ${GZ_SIM_LIBRARY_DIRS}
# )

# include_directories(
#   include
#   ${GAZEBO_INCLUDE_DIRS}
#   ${gz_sim_vendor_INCLUDE_DIRS}
#   ${GZ_SIM_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )

# # Drive node

# # Drive node
# set(EXEC_NAME "CTurtlebot3Drive")
# add_executable(${EXEC_NAME}
#   src/CTurtlebot3Drive.cpp
#   src/CTurtlebot3Sensor.cpp
#   src/CWallFollowingController.cpp
#   src/CPIDController.cpp
# )

# # (keep the rest as-is)
# target_link_libraries(${EXEC_NAME}
#   ${geometry_msgs_TARGETS}
#   ${nav_msgs_TARGETS}
#   ${sensor_msgs_TARGETS}
#   rclcpp::rclcpp
#   tf2::tf2
# )
# ament_target_dependencies(${EXEC_NAME}
#   rclcpp geometry_msgs nav_msgs sensor_msgs tf2
# )


# # Obstacles
# set(OBSTACLE_LIBS obstacles obstacle1 obstacle2)
# foreach(lib ${OBSTACLE_LIBS})
#   add_library(${lib} SHARED src/${lib}.cpp)
#   target_link_libraries(${lib}
#     PUBLIC
#     gz-sim::gz-sim
#     gz-math::gz-math
#   )
# endforeach()

# # 🔴 Red detector — standalone executable (only one target!)
# add_executable(red_detector src/red_detector.cpp)
# ament_target_dependencies(red_detector
#   rclcpp
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   image_transport
#   cv_bridge
# )
# target_link_libraries(red_detector ${OpenCV_LIBS})


# # -------------------- Install --------------------
# install(TARGETS
#   ${EXEC_NAME}
#   red_detector
#   obstacles
#   obstacle1
#   obstacle2
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(DIRECTORY launch models params rviz urdf worlds
#   DESTINATION share/${PROJECT_NAME}/
# )

# install(DIRECTORY include/
#   DESTINATION include/
# )

# # -------------------- Ament export --------------------
# ament_export_include_directories(include)
# ament_export_dependencies(
#   gazebo_ros_pkgs
#   geometry_msgs
#   nav_msgs
#   rclcpp
#   sensor_msgs
#   std_msgs
#   image_transport
#   cv_bridge
#   tf2
#   OpenCV
# )
# ament_package()