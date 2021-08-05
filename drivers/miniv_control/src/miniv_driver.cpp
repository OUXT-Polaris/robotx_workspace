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

#include <miniv_control/constants.hpp>
#include <miniv_control/miniv_driver.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>

#include <string>
#include <memory>

namespace miniv_control
{
MiniVDriver::MiniVDriver(
  const std::string & thruster_ip_address,
  const int & thruster_port,
  bool enable_dummy)
: thruster_ip_address(thruster_ip_address),
  thruster_port(thruster_port),
  enable_dummy(enable_dummy)
{
  if (!enable_dummy) {
    boost::asio::io_service io_service;
    tcp_client_ = std::make_unique<tcp_sender::TcpClient>(
      io_service, rclcpp::get_logger("MiniVHardware"));
    tcp_client_->connect(thruster_ip_address, thruster_port);
  }
}

bool MiniVDriver::sendCommand()
{
  nlohmann::json json;
  json["left"] = left_thrust_;
  json["right"] = right_thrust_;
  std::string message = json.dump();
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("MiniVHardware"), "sending command : " << message);
  if (enable_dummy) {
    return true;
  }
  return tcp_client_->send(message);
}

void MiniVDriver::setThrust(const Motor & motor, double thrust)
{
  switch (motor) {
    case Motor::THRUSTER:
      left_thrust_ = thrust;
      right_thrust_ = thrust;
      break;
    case Motor::THRUSTER_LEFT:
      left_thrust_ = thrust;
      break;
    case Motor::TURUSTER_RIGHT:
      right_thrust_ = thrust;
      break;
    default:
      break;
  }
}
}  // namespace miniv_control
