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

#ifndef HAZARD_STATUS_CONVETER__HAZARD_STATUS_CONVETER_HPP_
#define HAZARD_STATUS_CONVETER__HAZARD_STATUS_CONVETER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_vehicle_msgs/msg/engage.hpp>
#include <watchdog_system_msgs/msg/hazard_status_stamped.hpp>
#include <watchdog_system_msgs/msg/detail/hazard_status_stamped__struct.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace dummy_hazard_status_publisher{

using EngageMsg = autoware_auto_vehicle_msgs::msg::Engage;

using watchdog_system_msgs::msg::HazardStatus;
using watchdog_system_msgs::msg::HazardStatusStamped;

struct Param
{
  int update_rate;
};

struct HazardStatusParams{
  int level;
  bool emergency;
  bool emergency_holding;
  bool self_recoverable;
  double fault_time_after_engage;
};

struct MonitoringTopic {
  std::string name;
  HazardStatusParams hazard_status_params;
  rclcpp::Publisher<HazardStatusStamped>::SharedPtr pub_hazard_status_stamped_;
};


class DummyHazardStatusPublisher: public rclcpp::Node
{
public:
  DummyHazardStatusPublisher();


private:

  Param params_;
  std::vector<MonitoringTopic> monitoring_topics_;

  // Subscriber
  rclcpp::Subscription<EngageMsg>::SharedPtr sub_engage_;
  bool is_engaged_;
  void onEngage(EngageMsg::ConstSharedPtr msg);
  rclcpp::Time engaged_time_;

  // Publisher
  void loadMonitoringTopics();

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();


  void createDefaultDummyHazardStatus(HazardStatus & hazard_status);

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);
  void convertStrParamsToHazardStatusParams(
      rcl_interfaces::msg::SetParametersResult & result, HazardStatusParams & hazard_status_params, const std::vector<std::string> & str_params);
  void checkBoolParam(
      rcl_interfaces::msg::SetParametersResult & result, std::string str, std::string err_param_name);

};

} // namespace dummy_hazard_status_publisher


#endif //HAZARD_STATUS_CONVETER__HAZARD_STATUS_CONVETER_HPP_
