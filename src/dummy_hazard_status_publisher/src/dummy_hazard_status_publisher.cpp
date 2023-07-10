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

#include "dummy_hazard_status_publisher/dummy_hazard_status_publisher.hpp"
#include "dummy_hazard_status_publisher/update_param.hpp"

#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#define FMT_HEADER_ONLY
#include <fmt/format.h>
#include <functional>
#include <map>


namespace dummy_hazard_status_publisher
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

DummyHazardStatusPublisher::DummyHazardStatusPublisher(const rclcpp::NodeOptions & node_options)
  : Node("dummy_hazard_status_publisher", node_options)
{

  params_.update_rate = declare_parameter<int>("update_rate", 10);
  params_.hazard_status_params = declare_parameter<std::vector<std::string>>("hazard_status_params", std::vector<std::string>(4, ""));
  hazard_status_.level = 0;
  hazard_status_.emergency = false;
  hazard_status_.emergency_holding = false;
  hazard_status_.self_recoverable = false;

  pub_hazard_status_stamped_ = this->create_publisher<watchdog_system_msgs::msg::HazardStatusStamped>(
      "~/output/hazard_status", rclcpp::QoS(1));

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DummyHazardStatusPublisher::paramCallback, this, std::placeholders::_1));

  const auto update_period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&DummyHazardStatusPublisher::onTimer, this));

}

void DummyHazardStatusPublisher::onTimer()
{
  watchdog_system_msgs::msg::HazardStatusStamped hazard_status_stamped;
  hazard_status_stamped.stamp = this->now();
  hazard_status_stamped.status = hazard_status_;

  pub_hazard_status_stamped_->publish(hazard_status_stamped);

}



rcl_interfaces::msg::SetParametersResult DummyHazardStatusPublisher::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  const auto param_str = "hazard_status_params";

  for (const auto & param : parameters) {
    const auto param_name = param.get_name();
    if (param_name != param_str) {
      result.set__successful(false);
      result.set__reason("invalid parameter name");
      return result;
    }
    try {
      if (!tier4_autoware_utils::updateParam(parameters, param_name, params_.hazard_status_params)) {
      result.set__successful(false);
      result.set__reason("cannot set parameter by updateParam");
      };
    } catch (const rclcpp::exceptions::InvalidParametersException & e) {
        result.set__successful(false);
        result.set__reason(e.what());
        return result;
    }

    result = updateDummyHazardStatus(params_.hazard_status_params);
  }
  return result;
}

rcl_interfaces::msg::SetParametersResult DummyHazardStatusPublisher::updateDummyHazardStatus(
    const std::vector<std::string> & hazard_status_params
    )
{
  rcl_interfaces::msg::SetParametersResult result;

  if (hazard_status_params.size() != 4) {
    result.set__successful(false);
    result.set__reason("invalid parameter length, please set string param as follows: {level, emergency, emergency_holding, is_self_recoverable}");
    return result;
  }

  // check level argument
  try {
    const auto level_int = stoi(hazard_status_params[0]);
    if (0 > level_int || level_int > 3) {
      result.set__successful(false);
      result.set__reason("invalid level argument: level param must be from 0 to 3");
      return result;
    }
  }
  catch (const std::invalid_argument & e) {
    result.set__successful(false);
    result.set__reason("invalid level argument: cannot convert to int");
    return result;
  }
  // check other arguments
  std::map<int, std::string>indexes{{ 1, "emergency" }, { 2, "emergency_holding" }, { 3, "is_self_recoverable" }};
  for (const auto & index : indexes) {
    if (!checkBoolParam(result, params_.hazard_status_params[index.first], index.second)) {
      return result;
    };
  }
  getHazardStatusParam();
  RCLCPP_INFO(this->get_logger(), "setting parameters...level: %s, emergency: %s, emergency_holding: %s, is_self_recoverable: %s", hazard_status_params[0].c_str(), hazard_status_params[1].c_str(), hazard_status_params[2].c_str(), hazard_status_params[3].c_str());
  result.set__successful(true);
  return result;
}

bool DummyHazardStatusPublisher::checkBoolParam(
    rcl_interfaces::msg::SetParametersResult & result, std::string str, std::string err_param_name
)
{
  auto isBool = [](std::string str) {return (str == "false" || str == "true");};
  if (!isBool(str)) {
    result.set__successful(false);
    const auto err_msg = "invalid" + err_param_name + "argument: param must be bool";
    result.set__reason(err_msg);
    return false;
  }
  return true;
}

void DummyHazardStatusPublisher::getHazardStatusParam() {

  hazard_status_.level = std::stoi(params_.hazard_status_params[0]);
  hazard_status_.emergency = params_.hazard_status_params[1] == "true";
  hazard_status_.emergency_holding = params_.hazard_status_params[2] == "true";
  hazard_status_.self_recoverable = params_.hazard_status_params[3] == "true";

}



} // namespace dummy_hazard_status_publisher
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dummy_hazard_status_publisher::DummyHazardStatusPublisher);
