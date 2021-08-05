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

#include <geographic_conversion/geopath_converter_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace geographic_conversion
{
GeopathConverterComponent::GeopathConverterComponent(const rclcpp::NodeOptions & options)
: Node("geopath_converter_node", options)
{
  declare_parameter("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("~/path", 1);
  geopath_sub_ = this->create_subscription<geographic_msgs::msg::GeoPath>(
    "~/geopath", 1,
    std::bind(&GeopathConverterComponent::geoPathCallback, this, std::placeholders::_1));
}

nav_msgs::msg::Path GeopathConverterComponent::convert(geographic_msgs::msg::GeoPath path)
{
  nav_msgs::msg::Path ret;
  ret.header.stamp = path.header.stamp;
  ret.header.frame_id = map_frame_;
  for (auto itr = path.poses.begin(); itr != path.poses.end(); itr++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = itr->header.stamp;
    pose.pose.position = convertToPoint(itr->pose.position);
    pose.pose.orientation = itr->pose.orientation;
  }
  return ret;
}

geometry_msgs::msg::Point GeopathConverterComponent::convertToPoint(
  geographic_msgs::msg::GeoPoint point)
{
  geometry_msgs::msg::Point to;
  geodesy::UTMPoint utm_point;
  geodesy::fromMsg(point, utm_point);
  to.x = utm_point.northing;
  to.y = utm_point.easting * -1;
  to.z = utm_point.altitude;
  return to;
}

void GeopathConverterComponent::geoPathCallback(const geographic_msgs::msg::GeoPath::SharedPtr path)
{
  path_pub_->publish(convert(*path));
}
}  // namespace geographic_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(geographic_conversion::GeopathConverterComponent)
