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

#include <autoware_auto_system_msgs/msg/hazard_status_stamped.hpp>

#include "watchdog_system_msgs/msg/hazard_status_stamped.hpp"
#include <vector>

namespace hazard_status_converter {

class HazardStatusConverter : public rclcpp::Node
{
public:
  HazardStatusConverter();


private:

  // Subscriber
  rclcpp::Subscription<autoware_auto_system_msgs::msg::HazardStatusStamped>::SharedPtr sub_hazard_status_stamped_;
  void onHazardStatusStamped(const autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<watchdog_system_msgs::msg::HazardStatusStamped>::SharedPtr pub_hazard_status_stamped_;



  void loadModules();
  std::vector<std::string> non_self_recoverable_modules_;

  bool SelfRecoverableFromConfig(autoware_auto_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);
};

} // namespace hazard_status_stamped




#endif //HAZARD_STATUS_CONVETER__HAZARD_STATUS_CONVETER_HPP_
