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

#ifndef RAW_VEHICLE_CMD_CONVERTER__NODE_HPP_
#define RAW_VEHICLE_CMD_CONVERTER__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>

#include <memory>
#include <string>
#include <vector>

namespace raw_vehicle_cmd_converter
{
using AckermannControlCommand = autoware_auto_control_msgs::msg::AckermannControlCommand;
using tier4_vehicle_msgs::msg::ActuationCommandStamped;

class RawVehicleCommandConverterNode : public rclcpp::Node
{
public:
  explicit RawVehicleCommandConverterNode(const rclcpp::NodeOptions & node_options);

  //!< @brief topic publisher for low level vehicle command
  rclcpp::Publisher<ActuationCommandStamped>::SharedPtr pub_actuation_cmd_;
  //!< @brief subscriber for vehicle command
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;

  AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;

  void onControlCmd(const AckermannControlCommand::ConstSharedPtr msg);
  void publishActuationCmd();
};
}  // namespace raw_vehicle_cmd_converter

#endif  // RAW_VEHICLE_CMD_CONVERTER__NODE_HPP_
