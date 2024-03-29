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

#ifndef ECU__ECU_HPP_
#define ECU__ECU_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "watchdog_system_msgs/msg/hazard_status_stamped.hpp"
#include "watchdog_system_msgs/msg/switch_status.hpp"
#include "watchdog_system_msgs/msg/voter_state.hpp"

namespace supervisor{

using autoware_adapi_v1_msgs::msg::MrmState;
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_system_msgs::srv::OperateMrm;
using watchdog_system_msgs::msg::HazardStatus;
using watchdog_system_msgs::msg::HazardStatusStamped;
using watchdog_system_msgs::msg::SwitchStatus;
using watchdog_system_msgs::msg::VoterState;

const SwitchStatus::_ecu_type initial_selected_ecu_ = SwitchStatus::MAIN;

enum ecu_name {
  Main, Sub, Supervisor, None, Unknown
};

struct Ecu {
  ecu_name name;
  ecu_name external_ecu_name;
  ecu_name another_external_ecu_name;

  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_self_monitoring_;
  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_external_monitoring_;
  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_another_external_monitoring_;
  rclcpp::Subscription<MrmBehaviorStatus>::SharedPtr sub_mrm_comfortable_stop_status_;
  rclcpp::Subscription<MrmBehaviorStatus>::SharedPtr sub_mrm_sudden_stop_status_;

  HazardStatusStamped::ConstSharedPtr self_hazard_status_stamped_;
  HazardStatusStamped::ConstSharedPtr external_hazard_status_stamped_;
  HazardStatusStamped::ConstSharedPtr another_external_hazard_status_stamped_;

  rclcpp::Time stamp_self_hazard_status_;
  rclcpp::Time stamp_external_hazard_status_;
  rclcpp::Time stamp_another_external_hazard_status_;

  MrmBehaviorStatus::ConstSharedPtr mrm_comfortable_stop_status_;
  MrmBehaviorStatus::ConstSharedPtr mrm_sudden_stop_status_;

  rclcpp::CallbackGroup::SharedPtr client_mrm_comfortable_stop_group_;
  rclcpp::Client<OperateMrm>::SharedPtr client_mrm_comfortable_stop_;
  rclcpp::CallbackGroup::SharedPtr client_mrm_sudden_stop_group_;
  rclcpp::Client<OperateMrm>::SharedPtr client_mrm_sudden_stop_;

  // Control cmd
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr control_sub_;
  rclcpp::Subscription<GearCommand>::SharedPtr gear_sub_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr turn_indicators_sub_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr hazard_lights_sub_;

  AckermannControlCommand::ConstSharedPtr control_cmd_;
  GearCommand::ConstSharedPtr gear_cmd_;
  TurnIndicatorsCommand::ConstSharedPtr turn_indicators_cmd_;
  HazardLightsCommand::ConstSharedPtr hazard_lights_cmd_;



};


inline std::string convertEcuNameToString(
    const ecu_name name)
{
  if (name == Main) {
    return "Main";
  } else if (name == Sub) {
    return "Sub";
  } else if (name == Supervisor) {
    return "Supervisor";
  } else if (name == None) {
    return "None";
  }
  const auto msg = "invalid ECU name: " + std::to_string(name);
  throw std::runtime_error(msg);
}

inline ecu_name convertSwitchEcu(const SwitchStatus::_ecu_type ecu) {
  if (ecu == SwitchStatus::MAIN) {
    return Main;
  } else if (ecu == SwitchStatus::SUB) {
    return Sub;
  } else if (ecu == SwitchStatus::SUPERVISOR) {
    return Supervisor;
  }

  const auto msg = "invalid Switch ECU: " + std::to_string(ecu);
  throw std::runtime_error(msg);
}

inline ecu_name convertMrmEcu(const VoterState::_mrm_ecu_type ecu) {
  if (ecu == VoterState::NONE) {
    return None;
  } else if (ecu == VoterState::MAIN) {
    return Main;
  } else if (ecu == VoterState::SUB) {
    return Sub;
  } else if (ecu == VoterState::SUPERVISOR) {
    return Supervisor;
  } else if (ecu == VoterState::UNKNOWN) {
    return Unknown;
  }
  const auto msg = "invalid Mrm ECU: " + std::to_string(ecu);
  throw std::runtime_error(msg);
}

} // namespace supervisor

#endif //ECU__ECU_HPP_
