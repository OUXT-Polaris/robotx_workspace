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

#ifndef GEOGRAPHIC_CONVERSION__FIX_CONVERTER_COMPONENT_HPP_
#define GEOGRAPHIC_CONVERSION__FIX_CONVERTER_COMPONENT_HPP_

// Headers in ROS
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geodesy/utm.h>
#include <ctype.h>

// Headers in STL
#include <string>
#include <limits>
#include <cmath>

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_EXPORT __attribute__((dllexport))
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_EXPORT __declspec(dllexport)
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_BUILDING_DLL
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_PUBLIC \
  GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_EXPORT
#else
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_PUBLIC \
  GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_IMPORT
#endif
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_PUBLIC_TYPE \
  GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_PUBLIC
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_LOCAL
#else
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_EXPORT __attribute__((visibility("default")))
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_PUBLIC
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_LOCAL
#endif
#define GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

namespace geographic_conversion
{
class FixConverterComponent : public rclcpp::Node
{
public:
  GEOGRAPHIC_CONVERSION_FIX_CONVERTER_COMPONENT_PUBLIC
  explicit FixConverterComponent(const rclcpp::NodeOptions & options);

private:
  std::string map_frame_;
  std::string input_topic_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};
}  // namespace geographic_conversion

#endif  // GEOGRAPHIC_CONVERSION__FIX_CONVERTER_COMPONENT_HPP_
