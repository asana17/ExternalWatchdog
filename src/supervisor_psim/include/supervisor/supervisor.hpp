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

#include "supervisor/control_switch_interface.hpp"
#include "supervisor/ecu.hpp"
#include "supervisor/voter.hpp"


namespace supervisor{


struct Param
{
  int update_rate;
  double timeout_hazard_status;
  double service_timeout;
  std::string frame_id;
};

struct CurrentMrmStatus
{
  VoterState::_mrm_ecu_type mrm_ecu;
};

enum service_result {Success, Failure, Timeout, No_Response};

class SupervisorNode : public rclcpp::Node
{

public:
  SupervisorNode();

private:

  Ecu Main_, Sub_, Supervisor_;

  CurrentMrmStatus CurrentMrmStatus_;

  Voter Voter_;
  ControlSwitchInterface ControlSwitchInterface_;

  ecu_name switch_selected_ecu_;

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

  void onEcuControlCmd(const AckermannControlCommand::ConstSharedPtr msg, Ecu* ecu);
  void onEcuGearCmd(const GearCommand::ConstSharedPtr msg, Ecu* ecu);
  void onEcuTurnIndicatorsCmd(const TurnIndicatorsCommand::ConstSharedPtr msg, Ecu* ecu);
  void onEcuHazardLightsCmd(const HazardLightsCommand::ConstSharedPtr msg, Ecu* ecu);


  // Publisher
  void publishControlCommands();

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();
  bool isDataReady();
  bool is_data_ready_;
  bool isEcuDataReady();
  bool isControlDataReady();

  //Paramters
  Param param_;

  // Functions

  void callMrmBehavior(
      const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const;
  void cancelMrmBehavior(
      const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const;
  service_result callMrmService(
      const OperateMrm::Request::SharedPtr request, rclcpp::Client<OperateMrm>::SharedPtr mrm_client) const;

  void operateMrm();
  void cancelCurrentMrm(const VoterState::_mrm_ecu_type & mrm_ecu);
  MrmBehaviorStatus::_state_type checkMrmStatus(const VoterState::_mrm_ecu_type & mrm_ecu) const;

};

} // namespace supervisor

#endif //SUPERVISOR__SUPERVISOR_HPP_
