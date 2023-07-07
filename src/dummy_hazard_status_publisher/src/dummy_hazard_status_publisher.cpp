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

#include <functional>

namespace dummy_hazard_status_publisher
{

DummyHazardStatusPublisher::DummyHazardStatusPublisher(const rclcpp::NodeOptions & node_options)
  : Node("dummy_hazard_status_publisher", node_options)
{

  params_.update_rate = declare_parameter<int>("update_rate", 10);

  pub_hazard_status_stamped_ = this->create_publisher<watchdog_system_msgs::msg::HazardStatusStamped>(
      "~/output/hazard_status", rclcpp::QoS(1));

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

void DummyHazardStatusPublisher::createDummyHazardStatus()
{
  hazard_status_.level = 0;
  hazard_status_.emergency = false;
  hazard_status_.emergency_holding = false;
  hazard_status_.self_recoverable = false;
}


} // namespace dummy_hazard_status_publisher
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dummy_hazard_status_publisher::DummyHazardStatusPublisher);
