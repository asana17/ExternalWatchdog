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

#ifndef VOTER__VOTER_HPP_
#define VOTER__VOTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "watchdog_system_msgs/msg/hazard_status_stamped.hpp"
#include "watchdog_system_msgs/msg/switch_status.hpp"
#include "watchdog_system_msgs/msg/voter_state.hpp"

#include <functional>
#include <map>
#include <string>
#include <vector>


namespace Voter{

using autoware_adapi_v1_msgs::msg::MrmState;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_system_msgs::srv::OperateMrm;
using watchdog_system_msgs::msg::HazardStatus;
using watchdog_system_msgs::msg::HazardStatusStamped;
using watchdog_system_msgs::msg::SwitchStatus;
using watchdog_system_msgs::msg::VoterState;

enum ecu_name {
  Main, Sub, Supervisor, None, Unknown
};

struct Ecu {
  ecu_name name;
  ecu_name external_ecu_name;
  ecu_name another_external_ecu_name;

  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_self_monitoring_;
  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_external_monitoring_;
  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_another_external_monitoring_;
  rclcpp::Subscription<MrmBehaviorStatus>::SharedPtr sub_mrm_comfortable_stop_status_;
  rclcpp::Subscription<MrmBehaviorStatus>::SharedPtr sub_mrm_sudden_stop_status_;
  HazardStatusStamped::ConstSharedPtr self_hazard_status_stamped_;
  HazardStatusStamped::ConstSharedPtr external_hazard_status_stamped_;
  HazardStatusStamped::ConstSharedPtr another_external_hazard_status_stamped_;

  rclcpp::Time stamp_self_hazard_status_;
  rclcpp::Time stamp_external_hazard_status_;
  rclcpp::Time stamp_another_external_hazard_status_;

  MrmBehaviorStatus::ConstSharedPtr mrm_comfortable_stop_status_;
  MrmBehaviorStatus::ConstSharedPtr mrm_sudden_stop_status_;

  rclcpp::CallbackGroup::SharedPtr client_mrm_comfortable_stop_group_;
  rclcpp::Client<OperateMrm>::SharedPtr client_mrm_comfortable_stop_;
  rclcpp::CallbackGroup::SharedPtr client_mrm_sudden_stop_group_;
  rclcpp::Client<OperateMrm>::SharedPtr client_mrm_sudden_stop_;

};

struct ErrorStatus {
  bool is_emergency_;
  bool stop_operation_needed_;
  bool switch_needed_;
};

struct MrmOperation {
  VoterState::_fault_ecu_type fault_ecu;
  VoterState::_mrm_ecu_type mrm_ecu;
  VoterState::_comfortable_stop_after_switch_type comfortable_stop_after_switch;
};

struct Param
{
  int update_rate;
  double timeout_hazard_status;
};
class Voter : public rclcpp::Node
{

public:
  explicit Voter(const rclcpp::NodeOptions & node_options);

private:

  Ecu Main_, Sub_, Supervisor_;
  std::map<ecu_name, ErrorStatus> self_error_status_;
  std::map<ecu_name, ErrorStatus> external_error_status_;
  SwitchStatus switch_status_;

  // Subscriber

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  nav_msgs::msg::Odometry::ConstSharedPtr odom_;

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
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void onSwitchStatus(
      const SwitchStatus::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<SwitchStatus>::SharedPtr switch_status_pub_;

  VoterState voter_state_;
  MrmOperation mrm_operation_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  bool isDataReady();
  bool isEcuDataReady(const Ecu* ecu);

  //Paramters
  Param param_;

  void callMrmBehavior(
      const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const;
  void cancelMrmBehavior(
      const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const;


  // Functions

  std::string convertEcuNameToString(ecu_name name) const;

  bool is_emergency_;

  bool isStopped();
  void transitionTo(const int new_state);
  void transitionSwitchTo(const int new_ecu);
  void updateVoterState();
  ecu_name convertSwitchEcu(const SwitchStatus::_ecu_type ecu) const;
  VoterState::_fault_ecu_type convertFaultEcuName(const ecu_name name) const;
  VoterState::_mrm_ecu_type convertMrmEcuName(const ecu_name name) const;

  void checkError();
  void updateSelfErrorStatus();
  void updateExternalErrorStatus();
  bool checkExternalMonitoring(Ecu * ecu);
  void judgeMrmOperation();
  void getMrmOperationFromSelfMonitoring();
  void getMrmOperationFromExternalMonitoring();
  void getNoMrmOperation();
  void getMrmOperationInternal(std::map<ecu_name, ErrorStatus> & error_status_);
  void getMrmOperationMultipleEcuError();
  void operateMrm();
  void cancelCurrentMrm();

};

} // namespace Voter

#endif //VOTER__VOTER_HPP_
