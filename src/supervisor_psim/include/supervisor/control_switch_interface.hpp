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

#ifndef CONTROL_SWITCH_INTERFACE__CONTROL_SWITCH_INTERFACE_HPP_
#define CONTROL_SWITCH_INTERFACE__CONTROL_SWITCH_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "supervisor/ecu.hpp"

#include <string>

namespace supervisor{

class ControlSwitchInterface
{
public:

  ControlSwitchInterface();
  ~ControlSwitchInterface() = default;

  rclcpp::Publisher<AckermannControlCommand>::SharedPtr vehicle_control_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_pub_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_indicators_pub_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_lights_pub_;

  ecu_name getSelectedEcu() const;
  void publishControlToVehicle(const AckermannControlCommand::ConstSharedPtr msg);
  void publishGearToVehicle(const GearCommand::ConstSharedPtr msg);
  void publishTurnIndicatorsToVehicle(const TurnIndicatorsCommand::ConstSharedPtr msg);
  void publishHazardLightsToVehicle(const HazardLightsCommand::ConstSharedPtr msg);
  void changeSwitchTo(const SwitchStatus::_ecu_type);

private:

  SwitchStatus switch_status_;
};

} // namespace ControlSwitchInterface

#endif  // CONTROL_SWITCH_INTERFACE__CONTROL_SWITCH_INTERFACE_HPP_
