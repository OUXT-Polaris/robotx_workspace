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

#ifndef GEOGRAPHIC_CONVERSION__GEOPOSE_CONVERTER_COMPONENT_HPP_
#define GEOGRAPHIC_CONVERSION__GEOPOSE_CONVERTER_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_EXPORT __attribute__((dllexport))
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_EXPORT __declspec(dllexport)
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_BUILDING_DLL
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_PUBLIC \
  GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_EXPORT
#else
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_PUBLIC \
  GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_IMPORT
#endif
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_PUBLIC_TYPE \
  GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_PUBLIC
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_LOCAL
#else
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_LOCAL \
  __attribute__((visibility("hidden")))
#else
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_PUBLIC
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_LOCAL
#endif
#define GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

// Headers in ROS
#include <geographic_msgs/msg/geo_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ctype.h>
#include <geodesy/utm.h>

// Headers in STL
#include <string>
#include <limits>
#include <cmath>

namespace geographic_conversion
{
class GeoposeConverterComponent : public rclcpp::Node
{
public:
  GEOGRAPHIC_CONVERSION_GEOPOSE_CONVERTER_COMPONENT_PUBLIC
  explicit GeoposeConverterComponent(const rclcpp::NodeOptions & options);

private:
  std::string map_frame_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<geographic_msgs::msg::GeoPoseStamped>::SharedPtr geopose_sub_;
  geometry_msgs::msg::PoseStamped convert(geographic_msgs::msg::GeoPoseStamped geopose);
  void geoposeCallback(const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg);
};
}  // namespace geographic_conversion

#endif  // GEOGRAPHIC_CONVERSION__GEOPOSE_CONVERTER_COMPONENT_HPP_
