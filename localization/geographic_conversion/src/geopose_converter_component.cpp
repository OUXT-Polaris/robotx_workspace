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

#include <geographic_conversion/geopose_converter_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace geographic_conversion
{
GeoposeConverterComponent::GeoposeConverterComponent(const rclcpp::NodeOptions & options)
: Node("geopose_converter_node", options)
{
  declare_parameter("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", 1);
  geopose_sub_ = this->create_subscription<geographic_msgs::msg::GeoPoseStamped>(
    "~/geopose", 1,
    std::bind(&GeoposeConverterComponent::geoposeCallback, this, std::placeholders::_1));
}

void GeoposeConverterComponent::geoposeCallback(
  const geographic_msgs::msg::GeoPoseStamped::SharedPtr msg)
{
  pose_pub_->publish(convert(*msg));
}

geometry_msgs::msg::PoseStamped GeoposeConverterComponent::convert(
  geographic_msgs::msg::GeoPoseStamped geopose)
{
  geometry_msgs::msg::PoseStamped pose;
  geodesy::UTMPose utm_pose = geodesy::UTMPose(geopose.pose);
  pose.header.frame_id = map_frame_;
  pose.header.stamp = geopose.header.stamp;
  pose.pose.position.x = utm_pose.position.northing;
  pose.pose.position.y = utm_pose.position.easting * -1;
  pose.pose.position.z = utm_pose.position.altitude;
  pose.pose.orientation = utm_pose.orientation;
  return pose;
}
}  // namespace geographic_conversion

RCLCPP_COMPONENTS_REGISTER_NODE(geographic_conversion::GeoposeConverterComponent)
