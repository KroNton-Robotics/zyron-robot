cmake_minimum_required(VERSION 3.8)
project(zyron_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL libserial REQUIRED)

add_executable(zyron_serial_driver
  src/zyron_serial_driver.cpp
  src/zyron_serial_driver_node.cpp
)
target_include_directories(zyron_serial_driver PUBLIC
  include
  ${SERIAL_INCLUDE_DIRS}
)
target_link_libraries(zyron_serial_driver
  ${SERIAL_LDFLAGS}
)
ament_target_dependencies(zyron_serial_driver
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
)

install(TARGETS zyron_serial_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
