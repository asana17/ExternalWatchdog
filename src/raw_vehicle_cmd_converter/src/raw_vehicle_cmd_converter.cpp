//  Copyright 2023 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "raw_vehicle_cmd_converter/raw_vehicle_cmd_converter.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
RawVehicleCommandConverterNode::RawVehicleCommandConverterNode(
  const rclcpp::NodeOptions & node_options)
: Node("raw_vehicle_cmd_converter_node", node_options)
{
  using std::placeholders::_1;

  pub_actuation_cmd_ = create_publisher<ActuationCommandStamped>("~/output/actuation_cmd", 1);
  sub_control_cmd_ = create_subscription<AckermannControlCommand>(
    "~/input/control_cmd", 1, std::bind(&RawVehicleCommandConverterNode::onControlCmd, this, _1));
}

void RawVehicleCommandConverterNode::publishActuationCmd()
{
  if (!control_cmd_ptr_ ) {
    RCLCPP_WARN_EXPRESSION(
      get_logger(), "control_cmd pointer is null: %s",
      !control_cmd_ptr_ ? "cmd" : "");
    return;
  }
  double desired_accel_cmd = 0.0;
  double desired_brake_cmd = 0.0;
  double desired_steer_cmd = 0.0;
  ActuationCommandStamped actuation_cmd;
  const double acc = control_cmd_ptr_->longitudinal.acceleration;
  const double steer = control_cmd_ptr_->lateral.steering_tire_angle;

  desired_accel_cmd = (acc >= 0) ? acc : 0;
  if (acc < 0) {
    desired_brake_cmd = -acc;
  }
  desired_steer_cmd = steer;

  actuation_cmd.header.frame_id = "base_link";
  actuation_cmd.header.stamp = control_cmd_ptr_->stamp;
  actuation_cmd.actuation.accel_cmd = desired_accel_cmd;
  actuation_cmd.actuation.brake_cmd = desired_brake_cmd;
  actuation_cmd.actuation.steer_cmd = desired_steer_cmd;
  pub_actuation_cmd_->publish(actuation_cmd);
}

void RawVehicleCommandConverterNode::onControlCmd(const AckermannControlCommand::ConstSharedPtr msg)
{
  control_cmd_ptr_ = msg;
  publishActuationCmd();
}
}  // namespace raw_vehicle_cmd_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(raw_vehicle_cmd_converter::RawVehicleCommandConverterNode)
