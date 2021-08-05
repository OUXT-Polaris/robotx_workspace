cmake_minimum_required(VERSION 3.5)
project(geographic_conversion)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geodesy REQUIRED)

include_directories(
  include
)

add_library(fix_converter_component SHARED
  src/fix_converter_component.cpp
)
target_compile_definitions(fix_converter_component
  PRIVATE "nmea_to_geopose_nmea_to_geopose_COMPONENT_BUILDING_DLL")
ament_target_dependencies(fix_converter_component
  rclcpp
  rclcpp_components
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geodesy
  geographic_msgs)
rclcpp_components_register_nodes(fix_converter_component
  "geographic_conversion::FixConverterComponent")

add_library(geopath_converter_component SHARED
  src/geopath_converter_component.cpp
)
target_compile_definitions(geopath_converter_component
  PRIVATE "GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(geopath_converter_component
  rclcpp
  rclcpp_components
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geodesy
  geographic_msgs)
rclcpp_components_register_nodes(geopath_converter_component
  "geographic_conversion::GeoPathConverterComponent")

add_library(geopose_converter_component SHARED
  src/geopose_converter_component.cpp
)
target_compile_definitions(geopath_converter_component
  PRIVATE "GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_BUILDING_DLL")
ament_target_dependencies(geopose_converter_component
  rclcpp
  rclcpp_components
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geodesy
  geographic_msgs)
rclcpp_components_register_nodes(geopose_converter_component
  "geographic_conversion::GeoPoseConverterComponent")

install(TARGETS
  fix_converter_component
  geopose_converter_component
  geopath_converter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_libraries(fix_converter_component)
ament_export_libraries(geopath_converter_component)
ament_export_libraries(geopose_converter_component)

# install headers
install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()