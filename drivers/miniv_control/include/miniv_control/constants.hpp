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

#ifndef MINIV_CONTROL__CONSTANTS_HPP_
#define MINIV_CONTROL__CONSTANTS_HPP_

#include <cmath>
#include <cstdint>

namespace miniv_control
{
constexpr double PROTOCOL_VERSION = 1.0;
constexpr int DXL_HOME_POSITION = 511;  // value range:0 ~ 1023
constexpr double DXL_MAX_POSITION = 1023.0;
constexpr double DXL_MAX_POSITION_DEGREES = 300.0;
constexpr double TO_RADIANS = (DXL_MAX_POSITION_DEGREES / DXL_MAX_POSITION) * M_PI / 180.0;
constexpr double TO_DXL_POS = 1.0 / TO_RADIANS;
constexpr double TO_SPEED_REV_PER_MIN = 0.111;
constexpr double TO_SPEED_RAD_PER_MIN = TO_SPEED_REV_PER_MIN * 2.0 * M_PI;
constexpr double TO_SPEED_RAD_PER_SEC = TO_SPEED_RAD_PER_MIN / 60.0;
constexpr double TO_LOAD_PERCENT = 0.1;
constexpr double TO_VOLTAGE = 0.1;

// Dynamixel XW54-T260 address table
// Ref: https://emanual.robotis.com/docs/en/dxl/x/xw540-t260/
constexpr uint16_t ADDR_TORQUE_ENABLE = 64;
constexpr uint16_t ADDR_GOAL_POSITION = 116;
// constexpr uint16_t ADDR_MOVING_SPEED = 32;
constexpr uint16_t ADDR_PRESENT_POSITION = 132;
constexpr uint16_t ADDR_PRESENT_SPEED = 128;
// constexpr uint16_t ADDR_PRESENT_LOAD = 40;
// constexpr uint16_t ADDR_PRESENT_VOLTAGE = 42;
constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 146;

constexpr uint8_t LEFT_AZIMUTH_ID = 0;
constexpr uint8_t RIGHT_AZIMUTH_ID = 1;

}  // namespace miniv_control

#endif  // MINIV_CONTROL__CONSTANTS_HPP_
