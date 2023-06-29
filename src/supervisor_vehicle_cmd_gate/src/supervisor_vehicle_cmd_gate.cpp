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

  const auto stamp = this->now();

  AckermannControlCommand control_cmd;
  control_cmd = createEmergencyStopControlCmd();

  GearCommand gear;
  gear.stamp = stamp;

  TurnIndicatorsCommand turn_indicator;
  turn_indicator.stamp = stamp;
  turn_indicator.command = TurnIndicatorsCommand::NO_COMMAND;

  HazardLightsCommand hazard_light;
  hazard_light.stamp = stamp;
  hazard_light.command = HazardLightsCommand::ENABLE;

  control_cmd_pub_->publish(control_cmd);
  gear_cmd_pub_->publish(gear);
  turn_indicator_cmd_pub_->publish(turn_indicator);
  hazard_light_cmd_pub_->publish(hazard_light);

}

AckermannControlCommand SupervisorVehicleCmdGate::createEmergencyStopControlCmd()
{
  AckermannControlCommand cmd;
  const auto t = this->now();
  cmd.stamp = t;
  cmd.lateral.stamp = t;
  cmd.longitudinal.stamp = t;
  const auto filtered_command = filterControlCommand(emergency_commands_.control);
  cmd.lateral.steering_tire_angle = filtered_command.lateral.steering_tire_angle;
  cmd.lateral.steering_tire_rotation_rate = filtered_command.lateral.steering_tire_rotation_rate;
  cmd.longitudinal.speed = filtered_command.longitudinal.speed;
  cmd.longitudinal.acceleration = filtered_command.longitudinal.acceleration;

  return cmd;
}

AckermannControlCommand SupervisorVehicleCmdGate::filterControlCommand(const AckermannControlCommand & in)
{
  AckermannControlCommand out = in;
  const double dt = getDt();

  filter_.filterAll(dt, current_steer_, out);

  filter_.setPrevCmd(out);

  return out;
}


void SupervisorVehicleCmdGate::onSteering(SteeringReport::ConstSharedPtr msg)
{
  current_steer_ = msg->steering_tire_angle;
}


double SupervisorVehicleCmdGate::getDt()
{
  if (!prev_time_) {
    prev_time_ = std::make_shared<rclcpp::Time>(this->now());
    return 0.0;
  }

  const auto current_time = this->now();
  const auto dt = (current_time - *prev_time_).seconds();
  *prev_time_ = current_time;

  return dt;
}

} // namespace SupervisorVehicleCmdGate
