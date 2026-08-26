cmake_minimum_required(VERSION 3.8)
project(zyron_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# ── C++ hardware interface library ──────────────────────────────────────────
add_library(
  ${PROJECT_NAME}
  SHARED
  src/zyron_interface.cpp
  src/zyron_serial_driver.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  realtime_tools
  tf2
)

# LibSerial is a plain system library (libserial-dev), not an ament package
target_link_libraries(${PROJECT_NAME} serial)

# Register pluginlib plugin
pluginlib_export_plugin_description_file(hardware_interface zyron_interface.xml)

# ── Install C++ targets ──────────────────────────────────────────────────────
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ── Install launch and config directories ────────────────────────────────────
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ── Install Python package (zyron_control/) ──────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

# Install the Python node as an executable in lib/<pkg>/ so ros2 run finds it
install(
  PROGRAMS
    zyron_control/twist_to_twist_stamped.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Export ───────────────────────────────────────────────────────────────────
ament_export_include_directories(include)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
)

ament_package()
