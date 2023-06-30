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

#include "supervisor/can_switch_interface.hpp"
#include <string>

namespace supervisor{

CanSwitchInterface::CanSwitchInterface()
{
  switch_status_.ecu = initial_selected_ecu_;

}

ecu_name CanSwitchInterface::getSelectedEcu() const
{
  return convertSwitchEcu(switch_status_.ecu);
}

void CanSwitchInterface::publishCanToEcu(const can_msgs::msg::Frame::ConstSharedPtr msg, Ecu* ecu){
  ecu->can_pub_->publish(*msg);
}

void CanSwitchInterface::publishCanToVehicle(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  vehicle_can_pub_->publish(*msg);
}

void CanSwitchInterface::changeSwitchTo(const SwitchStatus::_ecu_type ecu)
{
  const auto ecu2string = [](const int ecu) {
    if (ecu == SwitchStatus::MAIN) {
      return "Main";
    }
    if (ecu == SwitchStatus::SUB) {
      return "Sub";
    }
    if (ecu == SwitchStatus::SUPERVISOR) {
      return "Supervisor";
    }
    const auto msg = "invalid ecu: " + std::to_string(ecu);
    throw std::runtime_error(msg);
  };

  ecu2string(ecu);

  /*RCLCPP_INFO(
    this->get_logger(), "Switch Status changing: %s -> %s", ecu2string(switch_status_.ecu),
    ecu2string(new_ecu));*/

  switch_status_.ecu = ecu;
}

} // namespace supervisor
