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

#ifndef SUPERVISOR__SUPERVISOR_HPP_
#define SUPERVISOR__SUPERVISOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>
#include "watchdog_system_msgs/msg/hazard_status_stamped.hpp"
#include "watchdog_system_msgs/msg/switch_status.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "supervisor/can_switch_interface.hpp"
#include "supervisor/ecu.hpp"
#include "supervisor/voter.hpp"


namespace supervisor{


struct Param
{
  int update_rate;
  double timeout_hazard_status;
  std::string frame_id;
};

struct CurrentMrmStatus
{
  VoterState::_mrm_ecu_type mrm_ecu;
};


class SupervisorNode : public rclcpp::Node
{

public:
  explicit SupervisorNode(const rclcpp::NodeOptions & node_options);

private:

  Ecu Main_, Sub_, Supervisor_;

  CurrentMrmStatus CurrentMrmStatus_;

  Voter Voter_;
  CanSwitchInterface CanSwitchInterface_;

  // Subscriber
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_report_;
  void onVelocityReport(const VelocityReport::ConstSharedPtr msg);


  void onSelfMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu);
  void onExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu);
  void onAnotherExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu);
  void onMrmComfortableStopStatus(
    const MrmBehaviorStatus::ConstSharedPtr msg, Ecu* ecu);
  void onMrmSuddenStopStatus(
    const MrmBehaviorStatus::ConstSharedPtr msg, Ecu* ecu);

  void onVehicleCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg);
  void onEcuCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg, Ecu* ecu);


  // Publisher

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();
  bool isDataReady();
  bool isEcuDataReady();

  //Paramters
  Param param_;

  // Functions

  void callMrmBehavior(
      const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const;
  void cancelMrmBehavior(
      const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const;

  void operateMrm();
  void cancelCurrentMrm();

};

} // namespace supervisor

#endif //SUPERVISOR__SUPERVISOR_HPP_
