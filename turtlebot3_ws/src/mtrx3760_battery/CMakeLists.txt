cmake_minimum_required(VERSION 3.8)
project(mtrx3760_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mtrx3760_interfaces REQUIRED)

# Uncomment for GUI support
# find_package(rqt_gui REQUIRED)
# find_package(rqt_gui_cpp REQUIRED)
# find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Include directories
include_directories(include)

# Battery Monitor Node
add_executable(battery_monitor_node
  src/battery_monitor_node.cpp
)
ament_target_dependencies(battery_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  mtrx3760_interfaces
)

# Terminal Display Node
add_executable(battery_terminal_display
  src/battery_terminal_display.cpp
)
ament_target_dependencies(battery_terminal_display
  rclcpp
  sensor_msgs
  std_msgs
  mtrx3760_interfaces
)

# GUI Display Node (uncomment when ready to implement)
# add_executable(battery_gui_display
#   src/battery_gui_display.cpp
#   src/battery_widget.cpp
# )
# ament_target_dependencies(battery_gui_display
#   rclcpp
#   sensor_msgs
#   std_msgs
#   mtrx3760_interfaces
#   rqt_gui
#   rqt_gui_cpp
# )
# target_link_libraries(battery_gui_display Qt5::Widgets)

# Install executables
install(TARGETS
  battery_monitor_node
  battery_terminal_display
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
