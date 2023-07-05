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

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include "supervisor/ecu.hpp"
#include "watchdog_system_msgs/msg/voter_state.hpp"


namespace supervisor{

using autoware_auto_vehicle_msgs::msg::VelocityReport;
using watchdog_system_msgs::msg::VoterState;

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

class Voter
{

public:

  Voter();
  ~Voter() = default;
  VoterState getVoterState() const;
  MrmOperation getMrmOperation() const;
  void onVelocityReport(const VelocityReport::ConstSharedPtr msg);

  void updateSelfErrorStatus(Ecu* ecu, const ecu_name switch_selected_ecu);
  void updateExternalErrorStatus(Ecu* ecu, const ecu_name switch_selected_ecu);
  void prepareMrmOperation(ecu_name switch_selected_ecu);


  VoterState::_mrm_ecu_type convertMrmEcuName(const ecu_name name) const;

private:

  std::map<ecu_name, ErrorStatus> self_error_status_;
  std::map<ecu_name, ErrorStatus> external_error_status_;

  // Subscriber
  VelocityReport::ConstSharedPtr velocity_report_;


  VoterState voter_state_;
  MrmOperation mrm_operation_;



  // Functions

  bool is_emergency_;

  void transitionTo(const int new_state);
  void judgeMrmOperation(const ecu_name switch_selected_ecu);
  void updateVoterState();
  VoterState::_fault_ecu_type convertFaultEcuName(const ecu_name name) const;

  bool checkExternalMonitoring(Ecu * ecu);
  void getMrmOperationFromSelfMonitoring(const ecu_name switch_selected_ecu);
  void getMrmOperationFromExternalMonitoring(const ecu_name switch_selected_ecu);
  void getNoMrmOperation();
  void getMrmOperationInternal(std::map<ecu_name, ErrorStatus> & error_status_, const ecu_name switch_selected_ecu);
  void getMrmOperationMultipleEcuError();

  bool isStopped() const;
  bool isEmergency(const HazardStatusStamped::ConstSharedPtr hazard_status) const;

};

} // namespace Voter

#endif //VOTER__VOTER_HPP_
