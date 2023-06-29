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

#ifndef SUPERVISOR_VEHICLE_CMD_GATE__SUPERVISOR_VEHICLE_CMD_GATE_HPP_
#define SUPERVISOR_VEHICLE_CMD_GATE__SUPERVISOR_VEHICLE_CMD_GATE_HPP_

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace SupervisorVehicleCmdGate {

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::SteeringReport;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;

struct Parameters
{

};


struct Commands
{
  AckermannControlCommand control;
  TurnIndicatorsCommand turn_indicator;
  HazardLightsCommand hazard_light;
  GearCommand gear;
  explicit Commands(const uint8_t & default_gear = GearCommand::PARK)
  {
    gear.command = default_gear;
  }
};

class SupervisorVehicleCmdGate : public rclcpp::Node
{
public:
  explicit SupervisorVehicleCmdGate(const rclcpp::NodeOptions & node_options);

private:

  Commands emergency_commands_;

  // Publisher
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<GearCommand>::SharedPtr gear_cmd_pub_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_indicator_cmd_pub_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_light_cmd_pub_;

  // Subscription

  rclcpp::Subscription<AckermannControlCommand>::SharedPtr emergency_control_cmd_sub_;

  void onEmergencyCtrlCmd(AckermannControlCommand::ConstSharedPtr msg);

  Parameters params_;

  void publishEmergencyStopControlCommands();


};

}

#endif //SUPERVISOR_VEHICLE_CMD_GATE__SUPERVISOR_VEHICLE_CMD_GATE_HPP_
