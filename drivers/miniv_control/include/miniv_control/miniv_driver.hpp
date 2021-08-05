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

#ifndef MINIV_CONTROL__MINIV_DRIVER_HPP_
#define MINIV_CONTROL__MINIV_DRIVER_HPP_

#include <tcp_sender/tcp_client.hpp>

#include <boost/optional.hpp>

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace miniv_control
{
enum class Motor
{
  THRUSTER_LEFT,
  TURUSTER_RIGHT,
  THRUSTER
};

class MiniVDriver
{
public:
  MiniVDriver(
    const std::string & thruster_ip_address,
    const int & thruster_port,
    bool enable_dummy = true);

  const std::string thruster_ip_address;
  const int thruster_port;
  const bool enable_dummy;
  void setThrust(const Motor & motor, double thrust);
  bool sendCommand();

private:
  double left_thrust_;
  double right_thrust_;
  std::unique_ptr<tcp_sender::TcpClient> tcp_client_;
};
}  // namespace miniv_control

#endif  // MINIV_CONTROL__MINIV_DRIVER_HPP_
