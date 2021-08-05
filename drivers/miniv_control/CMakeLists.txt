cmake_minimum_required(VERSION 3.5)
project(miniv_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tcp_sender REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)

set(LIBRARY_NAME "miniv_control")
add_library(
  ${LIBRARY_NAME}
  SHARED
  src/miniv_hardware.cpp
  src/miniv_driver.cpp
)
target_include_directories(
  ${LIBRARY_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${LIBRARY_NAME}
  hardware_interface
  pluginlib
  rclcpp
  tcp_sender
  ros2_control
)

pluginlib_export_plugin_description_file(hardware_interface miniv_hardware.xml)

install(
  TARGETS ${LIBRARY_NAME}
  DESTINATION lib
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${LIBRARY_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  tcp_sender
  ros2_control
)

ament_package()
