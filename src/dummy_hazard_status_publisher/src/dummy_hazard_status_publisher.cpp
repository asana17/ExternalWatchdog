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

DummyHazardStatusPublisher::DummyHazardStatusPublisher()
  : Node("dummy_hazard_status_publisher", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true).allow_undeclared_parameters(true))
{

  params_.update_rate = declare_parameter<int>("update_rate", 10);

  loadMonitoringTopics();

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&DummyHazardStatusPublisher::paramCallback, this, std::placeholders::_1));

  using std::placeholders::_1;

  sub_engage_ = create_subscription<EngageMsg>(
    "~/input/engage", rclcpp::QoS{1}, std::bind(&DummyHazardStatusPublisher::onEngage, this, _1));

  const auto update_period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&DummyHazardStatusPublisher::onTimer, this));

}

void DummyHazardStatusPublisher::loadMonitoringTopics()
{
  const auto param_key = std::string("monitoring_topics");
  const uint64_t depth = 3;
  const auto param_names = this->list_parameters({param_key}, depth).names;

  if (param_names.empty()) {
    RCLCPP_INFO(this->get_logger(), "no param in config: %s", param_key.c_str());
    return;
  }

  std::set<std::string> topic_names;

  for (const auto & param_name: param_names) {
    const auto split_names = split(param_name, '.');
    const auto & key_name = split_names.at(0);
    const auto & topic_name = split_names.at(1);

    const auto topic_name_with_prefix = fmt::format("{0}.{1}", key_name, topic_name);

    if (topic_names.count(topic_name_with_prefix) != 0) {
      continue; // duplicated param
    }

    topic_names.insert(topic_name_with_prefix);

    const auto level_key = topic_name_with_prefix + std::string(".level");
    std::string level_str, emergency_str, emergency_holding_str, self_recoverable_str, fault_time_after_engage_str;
    const auto emergency_key = topic_name_with_prefix + std::string(".emergency");
    const auto emergency_holding_key = topic_name_with_prefix + std::string(".emergency_holding");
    const auto self_recoverable_key = topic_name_with_prefix + std::string(".self_recoverable");
    const auto fault_time_after_engage_key = topic_name_with_prefix + std::string(".fault_time_after_engage");

    this->get_parameter_or(level_key, level_str, std::string("0"));
    this->get_parameter_or(emergency_key, emergency_str, std::string("false"));
    this->get_parameter_or(emergency_holding_key, emergency_holding_str, std::string("false"));
    this->get_parameter_or(self_recoverable_key, self_recoverable_str, std::string("true"));
    this->get_parameter_or(fault_time_after_engage_key, fault_time_after_engage_str, std::string("0.0"));

    int level = std::stoi(level_str);
    bool emergency{}, emergency_holding{}, self_recoverable{};
    std::istringstream(emergency_str) >> std::boolalpha >> emergency;
    std::istringstream(emergency_holding_str) >> std::boolalpha >> emergency_holding;
    std::istringstream(self_recoverable_str) >> std::boolalpha >> self_recoverable;
    double fault_time_after_engage = std::stod(fault_time_after_engage_str);

    monitoring_topics_.push_back({topic_name, {level, emergency, emergency_holding, self_recoverable, fault_time_after_engage}, this->create_publisher<HazardStatusStamped>(topic_name, rclcpp::QoS(1))});
  }

}

void DummyHazardStatusPublisher::onEngage(EngageMsg::ConstSharedPtr msg) {
  if (!is_engaged_ && msg->engage == true) {
    engaged_time_ = this->now();
    is_engaged_ = true;
  }
  if (is_engaged_ && msg->engage == false) {
    is_engaged_ = false;
  }
}

void DummyHazardStatusPublisher::onTimer()
{
  watchdog_system_msgs::msg::HazardStatusStamped hazard_status_stamped;
  hazard_status_stamped.stamp = this->now();

  for (const auto &monitoring_topic: monitoring_topics_) {
    double engaged_duration = 0.0;
    if (is_engaged_) {
      engaged_duration = (this->now() - engaged_time_).seconds();
    }
    if (monitoring_topic.hazard_status_params.fault_time_after_engage > engaged_duration) {
      createDefaultDummyHazardStatus(hazard_status_stamped.status);
    } else {
      hazard_status_stamped.status.level = monitoring_topic.hazard_status_params.level;
      hazard_status_stamped.status.emergency= monitoring_topic.hazard_status_params.emergency;
      hazard_status_stamped.status.emergency_holding= monitoring_topic.hazard_status_params.emergency_holding;
      hazard_status_stamped.status.self_recoverable= monitoring_topic.hazard_status_params.self_recoverable;
    }
    monitoring_topic.pub_hazard_status_stamped_->publish(hazard_status_stamped);
  }
}

void DummyHazardStatusPublisher::createDefaultDummyHazardStatus(HazardStatus & hazard_status)
{
  hazard_status.level = 0;
  hazard_status.emergency = false;
  hazard_status.emergency_holding = false;
  hazard_status.self_recoverable = true;
}

rcl_interfaces::msg::SetParametersResult DummyHazardStatusPublisher::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.set__successful(true);

  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();
    const auto split_names = split(param_name, '.');
    const auto topic_name = split_names.at(0);

    auto it = std::find_if(
        std::begin(monitoring_topics_), std::end(monitoring_topics_),
        [&topic_name](MonitoringTopic monitoring_topic){ return monitoring_topic.name == topic_name;});
    if (it == std::end(monitoring_topics_)) {
      result.set__successful(false);
      result.set__reason("no matching topic name");
      return result;
    }
    std::cout << "matching topic found" << std::endl;
    const auto hazard_status_params_with_prefix_str = topic_name + std::string(".hazard_status_params");
    std::vector<std::string> hazard_status_params_str;

    try {
      if (!tier4_autoware_utils::updateParam(parameters, hazard_status_params_with_prefix_str, hazard_status_params_str)) {
        result.set__successful(false);
        result.set__reason("updateParam failed");
        return result;
      };
    } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
        result.set__successful(false);
        result.set__reason(e.what());
        return result;
    }

    HazardStatusParams hazard_status_params;

    std::cout << "converting parameter" << std::endl;
    convertStrParamsToHazardStatusParams(result, hazard_status_params, hazard_status_params_str);
    if (result.successful == false) {
    std::cout << "converting parameter failed" << std::endl;
      return result;
    }

    RCLCPP_INFO(this->get_logger(), "changing hazard_status of %s, ...level: %s, emergency: %s, emergency_holding: %s, self_recoverable: %s", topic_name.c_str(), hazard_status_params_str[0].c_str(), hazard_status_params_str[1].c_str(), hazard_status_params_str[2].c_str(), hazard_status_params_str[3].c_str());

    it->hazard_status_params.level = hazard_status_params.level;
    it->hazard_status_params.emergency = hazard_status_params.emergency;
    it->hazard_status_params.emergency_holding = hazard_status_params.emergency_holding;
    it->hazard_status_params.self_recoverable = hazard_status_params.self_recoverable;
    it->hazard_status_params.fault_time_after_engage = 0.0;

  }
  return result;
}

void DummyHazardStatusPublisher::convertStrParamsToHazardStatusParams(
    rcl_interfaces::msg::SetParametersResult & result, HazardStatusParams & hazard_status_params, const std::vector<std::string> & str_params)
{
  if (str_params.size() != 4) {
    result.set__successful(false);
    result.set__reason("invalid parameter length, please set string param as follows: {level, emergency, emergency_holding, self_recoverable}");
    return;
  }


  // check level argument
  int level;
  try {
    level = stoi(str_params[0]);
    if (0 > level || level > 3) {
      result.set__successful(false);
      result.set__reason("invalid level argument: level param must be from 0 to 3");
      return;
    }
  } catch (const std::invalid_argument & e) {
    result.set__successful(false);
    result.set__reason("invalid level argument: cannot convert to int");
    return;
  }

  // check other arguments
  std::map<int, std::string>indexes{{ 1, "emergency" }, { 2, "emergency_holding" }, { 3, "self_recoverable" }};
  for (const auto & index : indexes) {
    std::cout << index.second << " parameter set" << std::endl;
    checkBoolParam(result, str_params[index.first], index.second);
    if (result.successful == false) {
      return;
    }
  }

  hazard_status_params.level = level;
  std::istringstream(str_params[1]) >> std::boolalpha >> hazard_status_params.emergency;
  std::istringstream(str_params[2]) >> std::boolalpha >> hazard_status_params.emergency_holding;
  std::istringstream(str_params[3]) >> std::boolalpha >> hazard_status_params.self_recoverable;
}

void DummyHazardStatusPublisher::checkBoolParam(
    rcl_interfaces::msg::SetParametersResult & result, std::string str, std::string err_param_name
)
{
  auto isBool = [](std::string str) {return (str == "false" || str == "true");};
  if (!isBool(str)) {
    result.set__successful(false);
    const auto err_msg = "invalid " + err_param_name + " argument: param must be bool";
    result.set__reason(err_msg);
    return;
  }
  return;
}

} // namespace dummy_hazard_status_publisher
