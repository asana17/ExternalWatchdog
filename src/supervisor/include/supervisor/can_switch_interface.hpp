// Copyright 2023 Tier IV, Inc.
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

#ifndef CAN_SWITCH_INTERFACE__CAN_SWITCH_INTERFACE_HPP_
#define CAN_SWITCH_INTERFACE__CAN_SWITCH_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <can_msgs/msg/frame.hpp>
#include "supervisor/ecu.hpp"

#include <string>

namespace supervisor{

class CanSwitchInterface
{
public:

  CanSwitchInterface();
  ~CanSwitchInterface() = default;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr vehicle_can_sub_;

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr vehicle_can_pub_;

  ecu_name getSelectedEcu() const;
  void publishCanToEcu(const can_msgs::msg::Frame::ConstSharedPtr msg, Ecu* ecu);
  void publishCanToVehicle(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void changeSwitchTo(const SwitchStatus::_ecu_type);

private:

  SwitchStatus switch_status_;
};

} // namespace CanSwitchInterface

#endif  // CAN_SWITCH_INTERFACE__CAN_SWITCH_INTERFACE_HPP_
