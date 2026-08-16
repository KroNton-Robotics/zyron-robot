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
find_package(realtime_tools REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
pkg_check_modules(SERIAL libserial REQUIRED)

add_library(zyron_control SHARED
  src/zyron_interface.cpp
  src/zyron_serial_driver.cpp
)
target_include_directories(zyron_control PUBLIC
  include
  ../zyron_protocol/include
  ${SERIAL_INCLUDE_DIRS}
)
target_link_libraries(zyron_control
  ${SERIAL_LDFLAGS}
  Threads::Threads
)
ament_target_dependencies(zyron_control
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  realtime_tools
)

install(PROGRAMS
  zyron_control/twist_to_twist_stamped.py
  DESTINATION ${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface zyron_interface.xml)

install(TARGETS zyron_control
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY ../zyron_protocol/include/
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
  realtime_tools
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_zyron_serial_driver test/test_zyron_serial_driver.cpp)
  if(TARGET test_zyron_serial_driver)
    target_include_directories(
      test_zyron_serial_driver PRIVATE include ../zyron_protocol/include ${SERIAL_INCLUDE_DIRS})
    target_link_libraries(
      test_zyron_serial_driver zyron_control ${SERIAL_LDFLAGS} Threads::Threads util)
    ament_target_dependencies(test_zyron_serial_driver realtime_tools)
  endif()
endif()

ament_package()
