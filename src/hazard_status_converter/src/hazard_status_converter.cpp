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

#include "hazard_status_converter/hazard_status_converter.hpp"

#include <functional>



namespace hazard_status_converter
{


std::vector<std::string> split(const std::string & str, const char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

HazardStatusConverter::HazardStatusConverter()
  : Node("hazard_status_converter", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{

  loadModules();

  using std::placeholders::_1;

  sub_hazard_status_stamped_ = create_subscription<autoware_auto_system_msgs::msg::HazardStatusStamped>(
    "~/input/hazard_status_stamped", rclcpp::QoS{1}, std::bind(&HazardStatusConverter::onHazardStatusStamped, this, _1));

  pub_hazard_status_stamped_ = create_publisher<watchdog_system_msgs::msg::HazardStatusStamped>("~/output/hazard_status_stamped", rclcpp::QoS{1});


}

void HazardStatusConverter::loadModules()
{
  const uint64_t depth = 3;
  const auto module_names = this->list_parameters({"non_self_recoverable_modules"}, depth).names;

  if (module_names.empty()) {
    RCLCPP_INFO(this->get_logger(), "no module in config: all self_recoverable flag is set to true");
    return;
  }

  for (const auto & module_name: module_names) {
    const auto split_names = split(module_name, '.');
    const auto & param_module_name = split_names.at(1);
    non_self_recoverable_modules_.push_back(param_module_name);
    RCLCPP_INFO(this->get_logger(), "non self_recoverable module: %s", param_module_name.c_str());
  }

}


void HazardStatusConverter::onHazardStatusStamped(const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg) {
  watchdog_system_msgs::msg::HazardStatusStamped watchdog_hazard_status_stamped;
  watchdog_hazard_status_stamped.stamp = msg->stamp;
  watchdog_hazard_status_stamped.status.level = msg->status.level;
  watchdog_hazard_status_stamped.status.emergency = msg->status.emergency;
  watchdog_hazard_status_stamped.status.emergency_holding = msg->status.emergency_holding;
  watchdog_hazard_status_stamped.status.diag_no_fault = msg->status.diag_no_fault;
  watchdog_hazard_status_stamped.status.diag_safe_fault = msg->status.diag_safe_fault;
  watchdog_hazard_status_stamped.status.diag_latent_fault = msg->status.diag_latent_fault;
  watchdog_hazard_status_stamped.status.diag_single_point_fault = msg->status.diag_single_point_fault;
  watchdog_hazard_status_stamped.status.self_recoverable = SelfRecoverableFromConfig(msg);

  pub_hazard_status_stamped_->publish(watchdog_hazard_status_stamped);
}

bool HazardStatusConverter::SelfRecoverableFromConfig(autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg) {

  if (non_self_recoverable_modules_.empty()) {
    return true;
  }

  for (const auto & diagnostic_status : msg->status.diag_latent_fault) {
    for (const auto & non_self_recoverable_module: non_self_recoverable_modules_) {
      if (diagnostic_status.name.find(non_self_recoverable_module) != std::string::npos) {
        return false;
      }
    }
  }
  for (const auto & diagnostic_status : msg->status.diag_single_point_fault) {
    for (const auto & non_self_recoverable_module: non_self_recoverable_modules_) {
      if (diagnostic_status.name.find(non_self_recoverable_module) != std::string::npos) {
        return false;
      }
    }
  }
  return true;
}

} // namespace hazard_status_converter
