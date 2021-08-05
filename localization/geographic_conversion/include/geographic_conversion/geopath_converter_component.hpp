// Copyright (c) 2019 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GEOGRAPHIC_CONVERSION__GEOPATH_CONVERTER_COMPONENT_HPP_
#define GEOGRAPHIC_CONVERSION__GEOPATH_CONVERTER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_EXPORT __attribute__((dllexport))
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_EXPORT __declspec(dllexport)
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_BUILDING_DLL
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_PUBLIC \
  GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_EXPORT
#else
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_PUBLIC \
  GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_IMPORT
#endif
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_PUBLIC_TYPE \
  GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_PUBLIC
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_LOCAL
#else
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_PUBLIC
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_LOCAL
#endif
#define GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

// Headers in ROS
#include <geographic_msgs/msg/geo_path.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ctype.h>
#include <geodesy/utm.h>

// Headers in STL
#include <string>
#include <limits>
#include <cmath>

namespace geographic_conversion
{
class GeopathConverterComponent : public rclcpp::Node
{
public:
  GEOGRAPHIC_CONVERSION_GEOPATH_CONVERTER_COMPONENT_PUBLIC
  explicit GeopathConverterComponent(const rclcpp::NodeOptions & options);

private:
  std::string map_frame_;
  nav_msgs::msg::Path convert(geographic_msgs::msg::GeoPath path);
  geometry_msgs::msg::Point convertToPoint(geographic_msgs::msg::GeoPoint point);
  void geoPathCallback(const geographic_msgs::msg::GeoPath::SharedPtr path);
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPath>::SharedPtr geopath_sub_;
};
}  // namespace geographic_conversion

#endif  // GEOGRAPHIC_CONVERSION__GEOPATH_CONVERTER_COMPONENT_HPP_
