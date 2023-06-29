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
#include "watchdog_system_msgs/msg/switch_status.hpp"

#include <string>

namespace CanSwitchInterface {

using watchdog_system_msgs::msg::SwitchStatus;

enum ecu_name {
  Main, Sub, Supervisor
};

struct Ecu {
  ecu_name name;

  // Subscriber
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
  can_msgs::msg::Frame::ConstSharedPtr can_msg_;

  // Publisher
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
};

class CanSwitchInterface : public rclcpp::Node
{
public:
  explicit CanSwitchInterface(const rclcpp::NodeOptions & node_options);

private:
  // Parameter
  std::string frame_id_;

  Ecu Main_, Sub_, Supervisor_;

  can_msgs::msg::Frame::ConstSharedPtr selected_can_frame_;

  // Subscriber
  rclcpp::Subscription<SwitchStatus>::SharedPtr switch_status_sub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  SwitchStatus::SharedPtr switch_status_;
  can_msgs::msg::Frame::ConstSharedPtr can_msg_;

  // Publisher
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;

  ecu_name convertSwitchEcu(const SwitchStatus::_ecu_type ecu) const;
  void onSwitchStatus(const SwitchStatus::SharedPtr msg);
  void onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void onEcuCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg, Ecu* ecu);
  void onSupervisorCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg);

};

} // namespace CanSwitchInterface

#endif  // CAN_SWITCH_INTERFACE__CAN_SWITCH_INTERFACE_HPP_
