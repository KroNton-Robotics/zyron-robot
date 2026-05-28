cmake_minimum_required(VERSION 3.8)
project(zyron_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(zyron_control SHARED
  src/zyron_interface.cpp
)
target_include_directories(zyron_control PUBLIC
  include
)
ament_target_dependencies(zyron_control
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  sensor_msgs
  std_msgs
)

install(PROGRAMS
  zyron_control/twist_to_twist_stamped.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface zyron_interface.xml)

install(TARGETS zyron_control
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(zyron_control)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

ament_package()
