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

#include <geographic_conversion/fix_converter_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace geographic_conversion
{
FixConverterComponent::FixConverterComponent(const rclcpp::NodeOptions & options)
: Node("fix_converter_node", options)
{
  declare_parameter("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("~/position", 1);
  fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "~/fix", 1, std::bind(&FixConverterComponent::fixCallback, this, std::placeholders::_1));
}

void FixConverterComponent::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  geographic_msgs::msg::GeoPoint geopoint;
  geopoint.longitude = msg->longitude;
  geopoint.latitude = msg->latitude;
  geopoint.altitude = msg->altitude;
  geodesy::UTMPoint utm_point = geodesy::UTMPoint(geopoint);
  geometry_msgs::msg::PointStamped point;
  point.header.frame_id = map_frame_;
  point.header.stamp = msg->header.stamp;
  point.point.x = utm_point.northing;
  point.point.y = utm_point.easting * -1;
  point.point.z = utm_point.altitude;
  point_pub_->publish(point);
}
}  // namespace geographic_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(geographic_conversion::FixConverterComponent)
