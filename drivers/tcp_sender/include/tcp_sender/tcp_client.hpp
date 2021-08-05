// Copyright (c) 2021 OUXT Polaris
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

#ifndef TCP_SENDER__TCP_CLIENT_HPP_
#define TCP_SENDER__TCP_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <string>

namespace tcp_sender
{
class TcpClient
{
public:
  explicit TcpClient(
    boost::asio::io_service & io_service,
    const rclcpp::Logger & logger);
  void connect(const std::string & address, const int & port);
  bool send(const std::string & message);

private:
  boost::asio::io_service & io_service_;
  boost::asio::ip::tcp::socket socket_;
  rclcpp::Logger logger_;
};
}  // namespace tcp_sender

#endif  // TCP_SENDER__TCP_CLIENT_HPP_
