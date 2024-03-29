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

#include "can_switch_interface/can_switch_interface.hpp"
#include <string>

namespace CanSwitchInterface {

CanSwitchInterface::CanSwitchInterface(const rclcpp::NodeOptions & node_options)
: Node("can_interface", node_options)
{
  using std::placeholders::_1;

  frame_id_ = declare_parameter("frame_id", "base_link");

  // Subscriber
  switch_status_sub_ = create_subscription<watchdog_system_msgs::msg::SwitchStatus>(
    "~/input/switch_status", rclcpp::QoS(1), std::bind(&CanSwitchInterface::onSwitchStatus, this, _1));
  can_sub_ = create_subscription<can_msgs::msg::Frame>("/gsm8/from_can_bus", rclcpp::QoS(500), std::bind(&CanSwitchInterface::onCanFrame, this, _1));

  // Publisher
  can_pub_ = create_publisher<can_msgs::msg::Frame>("/gsm8/to_can_bus", rclcpp::QoS(500));

  // Initialize each ECUs

  Main_.name = Main;
  Sub_.name = Sub;
  Supervisor_.name = Supervisor;

  for (auto ecu : {&Main_, &Sub_}) {
    std::string topic_prefix;

    if (ecu->name == Main) {
      topic_prefix = "~/main";
    } else if (ecu->name == Sub) {
      topic_prefix = "~/sub";
    }

    // Subscriber

    ecu->can_sub_=
      create_subscription<can_msgs::msg::Frame>(
        topic_prefix + "/from_can_bus", rclcpp::QoS{500},
        [ecu, this](const can_msgs::msg::Frame::ConstSharedPtr msg) {
          CanSwitchInterface::onEcuCanFrame(msg, ecu);
        });

    // Publisher
    ecu->can_pub_= this->create_publisher<can_msgs::msg::Frame>(
        topic_prefix + "/to_can_bus", rclcpp::QoS(500));

  }

  Supervisor_.can_sub_ =
    create_subscription<can_msgs::msg::Frame>(
        "~/supervisor/from_can_bus", rclcpp::QoS(500), std::bind(&CanSwitchInterface::onSupervisorCanFrame, this,
          std::placeholders::_1));
  Supervisor_.can_pub_= this->create_publisher<can_msgs::msg::Frame>(
        "~/supervisor/to_can_bus", rclcpp::QoS(500));

}

void CanSwitchInterface::onSwitchStatus(const SwitchStatus::SharedPtr msg)
{
  switch_status_ = msg;
}


ecu_name CanSwitchInterface::convertSwitchEcu(const SwitchStatus::_ecu_type ecu) const
{

  if (ecu == SwitchStatus::MAIN) {
    return Main;
  } else if (ecu == SwitchStatus::SUB) {
    return Sub;
  } else if (ecu == SwitchStatus::SUPERVISOR) {
    return Supervisor;
  }

  const auto msg = "invalid SwitchStatus: " + std::to_string(ecu);
  throw std::runtime_error(msg);
}


void CanSwitchInterface::onCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  can_msg_ = msg;

  for (auto ecu : {&Main_, &Sub_, &Supervisor_}) {
    ecu->can_pub_->publish(*can_msg_);
  }
}


// Main or Sub
void CanSwitchInterface::onEcuCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->can_msg_ = msg;

  if (convertSwitchEcu(switch_status_->ecu) == ecu->name) {
    can_pub_->publish(*msg);
  }

}

// Supervisor
void CanSwitchInterface::onSupervisorCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg) {
  Supervisor_.can_msg_ = msg;
  switch_status_->ecu = SwitchStatus::SUPERVISOR;
  can_pub_->publish(*msg);
}

} // namespace CanSwitchInterface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CanSwitchInterface::CanSwitchInterface)
