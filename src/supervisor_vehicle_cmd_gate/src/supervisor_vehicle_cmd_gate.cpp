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

#include "supervisor_vehicle_cmd_gate/supervisor_vehicle_cmd_gate.hpp"
#include <functional>

namespace SupervisorVehicleCmdGate {

SupervisorVehicleCmdGate::SupervisorVehicleCmdGate(const rclcpp::NodeOptions & node_options)
: Node("supervisor_vehicle_cmd_gate", node_options)
{

  using std::placeholders::_1;

  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();

  control_cmd_pub_ =
    this->create_publisher<AckermannControlCommand>("output/control_cmd", durable_qos);
  gear_cmd_pub_ = this->create_publisher<GearCommand>("output/gear_cmd", durable_qos);
  turn_indicator_cmd_pub_ =
    this->create_publisher<TurnIndicatorsCommand>("output/turn_indicators_cmd", durable_qos);
  hazard_light_cmd_pub_ =
    this->create_publisher<HazardLightsCommand>("output/hazard_lights_cmd", durable_qos);

  emergency_control_cmd_sub_ = this->create_subscription<AckermannControlCommand>(
    "input/emergency/control_cmd", 1, std::bind(&SupervisorVehicleCmdGate::onEmergencyCtrlCmd, this, _1));

}


void SupervisorVehicleCmdGate::onEmergencyCtrlCmd(AckermannControlCommand::ConstSharedPtr msg)
{
  emergency_commands_.control = *msg;

  publishEmergencyStopControlCommands();
}

void SupervisorVehicleCmdGate::publishEmergencyStopControlCommands()
{

}

} // namespace SupervisorVehicleCmdGate
