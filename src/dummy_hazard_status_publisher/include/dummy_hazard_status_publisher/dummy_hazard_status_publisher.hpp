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

#include "watchdog_system_msgs/msg/hazard_status_stamped.hpp"
#include <vector>

namespace dummy_hazard_status_publisher{

struct Param
{
  int update_rate;
};


class DummyHazardStatusPublisher: public rclcpp::Node
{
public:
  explicit DummyHazardStatusPublisher(const rclcpp::NodeOptions & node_options);


private:

  Param params_;

  // Publisher
  rclcpp::Publisher<watchdog_system_msgs::msg::HazardStatusStamped>::SharedPtr pub_hazard_status_stamped_;

  watchdog_system_msgs::msg::HazardStatus hazard_status_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  void createDummyHazardStatus();

};

} // namespace dummy_hazard_status_publisher


#endif //HAZARD_STATUS_CONVETER__HAZARD_STATUS_CONVETER_HPP_
