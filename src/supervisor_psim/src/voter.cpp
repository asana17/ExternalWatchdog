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

#include "supervisor/voter.hpp"
#include <vector>
#include <iostream>

namespace supervisor{

Voter::Voter()
{

  for (auto ecu_name : {Main, Sub, Supervisor}) {
    self_error_status_[ecu_name] = ErrorStatus{false, false, false};
    external_error_status_[ecu_name] = ErrorStatus{false, false, false};
  }

  voter_state_.state = VoterState::NORMAL;
  voter_state_.external_detected = false;
  voter_state_.comfortable_stop_after_switch = false;
  mrm_operation_.fault_ecu = VoterState::NONE;
  mrm_operation_.mrm_ecu = None;
  mrm_operation_.comfortable_stop_after_switch = false;

}

VoterState Voter::getVoterState() const
{
  return voter_state_;
}

MrmOperation Voter::getMrmOperation() const
{
  return mrm_operation_;
}

void Voter::onVelocityReport(const VelocityReport::ConstSharedPtr msg)
{
  velocity_report_ = msg;
}

void Voter::transitionTo(const int new_state)
{

  const auto state2string = [](const int state) {
    if (state == VoterState::NORMAL) {
      return "NORMAL";
    }
    if (state == VoterState::SUPERVISOR_STOP) {
      return "SUPERVISOR_STOP";
    }
    if (state == VoterState::COMFORTABLE_STOP) {
      return "COMFORTABLE_STOP";
    }
    if (state == VoterState::MRM_SUCCEEDED) {
      return "MRM_SUCCEEDED";
    }
    if (state == VoterState::MRM_FAILED) {
      return "MRM_FAILED";
    }

    const auto msg = "invalid state: " + std::to_string(state);
    throw std::runtime_error(msg);
  };

/*  RCLCPP_INFO(
    this->get_logger(), "Voter State changed: %s -> %s", state2string(voter_state_.state),
    state2string(new_state)); */
  std::cout << "Voter State changed: " << state2string(voter_state_.state) << " -> " << state2string(new_state) << std::endl;
  state2string(new_state);

  voter_state_.state = new_state;
}


VoterState::_fault_ecu_type Voter::convertFaultEcuName(const ecu_name name) const
{
  if (name == Main) {
    return VoterState::MAIN;
  } else if (name == Sub) {
    return VoterState::SUB;
  } else if (name == Supervisor) {
    return VoterState::SUPERVISOR;
  } else if (name == Unknown) {
    return VoterState::UNKNOWN;
  } else if (name == None) {
    return VoterState::NONE;
  }

  const auto msg = "invalid Fault ecu_name: " + std::to_string(name);
  throw std::runtime_error(msg);
}
VoterState::_fault_ecu_type Voter::convertMrmEcuName(const ecu_name name) const
{
  if (name == Main) {
    return VoterState::MAIN;
  } else if (name == Sub) {
    return VoterState::SUB;
  } else if (name == Supervisor) {
    return VoterState::SUPERVISOR;
  } else if (name == None) {
    return VoterState::NONE;
  }

  const auto msg = "invalid Mrm ecu_name: " + std::to_string(name);
  throw std::runtime_error(msg);
}


void Voter::updateSelfErrorStatus(Ecu* ecu, const ecu_name switch_selected_ecu)
{
  if (isEmergency(ecu->self_hazard_status_stamped_)) {
    std::cout << "emergency!" << std::endl;
    std::cout << "ecu_name " << ecu->name << std::endl;
    std::cout << "switch_ecu_name " << switch_selected_ecu << std::endl;

    if (switch_selected_ecu == ecu->name) {
      const bool is_self_recoverable = ecu->self_hazard_status_stamped_->status.self_recoverable;
      // if self_recoverable, emergency_handler already called stop operator
      if (is_self_recoverable) {
        self_error_status_[ecu->name].is_emergency_ = true;
        self_error_status_[ecu->name].stop_operation_needed_ = false;
        self_error_status_[ecu->name].switch_needed_ = false;

      } else {
        // if non self recoverable, switch and emergency stop in supervisor
        self_error_status_[ecu->name].is_emergency_ = true;
        self_error_status_[ecu->name].stop_operation_needed_ = true;
        self_error_status_[ecu->name].switch_needed_ = true;
      }
    }
    else {
      // emergency occured on non switch selected ecu
      // operate comfortable stop on selected_ecu
      if (switch_selected_ecu != initial_selected_ecu_) {
        // already switched: non self_recoverable && switch needed fault already occuredand called stop operator
        // do nothing
        self_error_status_[ecu->name].is_emergency_ = true;
        self_error_status_[ecu->name].stop_operation_needed_ = false;
        self_error_status_[ecu->name].switch_needed_ = false;
      } else {
        // comfortable stop in selected_ecu
        self_error_status_[ecu->name].is_emergency_ = true;
        self_error_status_[ecu->name].stop_operation_needed_ = true;
        self_error_status_[ecu->name].switch_needed_ = false;
      }
    }
  } else { // is_emergency == false
    self_error_status_[ecu->name].is_emergency_ = false;
    self_error_status_[ecu->name].stop_operation_needed_ = false;
    self_error_status_[ecu->name].switch_needed_ = false;

  }
}

void Voter::updateExternalErrorStatus(Ecu* ecu, const ecu_name switch_selected_ecu)
{

  const bool is_emergency = checkExternalMonitoring(ecu);
  if (is_emergency) {
    if (switch_selected_ecu == ecu->name) {
      //switch and emergency stop in supervisor
      self_error_status_[ecu->name].is_emergency_ = true;
      self_error_status_[ecu->name].is_emergency_ = true;
      self_error_status_[ecu->name].stop_operation_needed_ = true;
      self_error_status_[ecu->name].switch_needed_ = true;
    } else {
      if (switch_selected_ecu != initial_selected_ecu_) {
        // already switched: switch needed fault already occuredand called stop operator
        // do nothing
        self_error_status_[ecu->name].is_emergency_ = true;
        self_error_status_[ecu->name].stop_operation_needed_ = false;
        self_error_status_[ecu->name].switch_needed_ = false;
      } else {
        // comfortable stop in selected_ecu
        self_error_status_[ecu->name].is_emergency_ = true;
        self_error_status_[ecu->name].stop_operation_needed_ = true;
        self_error_status_[ecu->name].switch_needed_ = false;
      }
    }
  } else { // is_emergency == false
    self_error_status_[ecu->name].is_emergency_ = false;
    self_error_status_[ecu->name].stop_operation_needed_ = false;
    self_error_status_[ecu->name].switch_needed_ = false;
  }
}

bool Voter::checkExternalMonitoring(Ecu * ecu) {
  return isEmergency(ecu->external_hazard_status_stamped_) && isEmergency(ecu->another_external_hazard_status_stamped_);
}

void Voter::prepareMrmOperation(ecu_name switch_selected_ecu) {
  judgeMrmOperation(switch_selected_ecu);
  updateVoterState();
}

void Voter::judgeMrmOperation(ecu_name switch_selected_ecu) {
  voter_state_.external_detected = false;
  getMrmOperationFromExternalMonitoring(switch_selected_ecu);
  if (voter_state_.external_detected) {
    std::cout << "external_detected!" << std::endl;
  }
  if (!voter_state_.external_detected) {
    getMrmOperationFromSelfMonitoring(switch_selected_ecu);
    if (mrm_operation_.mrm_ecu != VoterState::NONE) {
      std::cout << "interal_detected!" << std::endl;
      std::cout << "mrm_operation" << std::endl;
      std::cout << "fault_ecu " << mrm_operation_.fault_ecu << std::endl;
      std::cout << "mrm_ecu " << mrm_operation_.mrm_ecu << std::endl;
      std::cout << "comfortable_stop_after_switch" << mrm_operation_.comfortable_stop_after_switch << std::endl;
      std::cout << "mrm_operation_end" << std::endl;
    }
  }
}

void Voter::getMrmOperationFromSelfMonitoring(ecu_name switch_selected_ecu)
{
  int self_monitoring_result = int(self_error_status_[Main].is_emergency_) + int(self_error_status_[Sub].is_emergency_) + int(self_error_status_[Supervisor].is_emergency_);
  if (self_monitoring_result == 0) {
    getNoMrmOperation();
  } else if (self_monitoring_result == 1) {
    getMrmOperationInternal(self_error_status_, switch_selected_ecu);
  } else {
    // self error detection on multiple ECUs
    getMrmOperationMultipleEcuError();
  }
}

void Voter::getMrmOperationFromExternalMonitoring(ecu_name switch_selected_ecu)
{
  int external_monitoring_result = int(external_error_status_[Main].is_emergency_) + int(external_error_status_[Sub].is_emergency_) + int(external_error_status_[Supervisor].is_emergency_);
  if (external_monitoring_result == 0) {
    getNoMrmOperation();
  } else {
    voter_state_.external_detected = true;
    if (external_monitoring_result == 1) {
      getMrmOperationInternal(external_error_status_, switch_selected_ecu);
    } else if (external_monitoring_result > 1) {
      getMrmOperationMultipleEcuError();
    }
  }
}

void Voter::getNoMrmOperation()
{
  mrm_operation_.fault_ecu = VoterState::NONE;
  mrm_operation_.mrm_ecu = VoterState::NONE;
  mrm_operation_.comfortable_stop_after_switch = false;
}

void Voter::getMrmOperationInternal(std::map<ecu_name, ErrorStatus> & error_status_, ecu_name switch_selected_ecu)
{
  // only single ECU is emergency
  for (ecu_name e : {Main, Sub, Supervisor}) {
    if (error_status_[e].is_emergency_) {
      mrm_operation_.fault_ecu = convertFaultEcuName(e);
      if (!error_status_[e].stop_operation_needed_) {
        mrm_operation_.mrm_ecu = VoterState::NONE;
        mrm_operation_.comfortable_stop_after_switch = false;
      } else {
        if (self_error_status_[e].switch_needed_) {
          mrm_operation_.mrm_ecu = VoterState::SUPERVISOR;
          mrm_operation_.comfortable_stop_after_switch = true;
        } else {
          mrm_operation_.mrm_ecu = convertMrmEcuName(switch_selected_ecu);
          mrm_operation_.comfortable_stop_after_switch = false;
        }
      }
    }
  }
}

void Voter::getMrmOperationMultipleEcuError()
{
 // RCLCPP_ERROR(this->get_logger(), "Self error detection on multiple ECUs: operate MRM on Supervisor");
  mrm_operation_.fault_ecu = VoterState::UNKNOWN;
  mrm_operation_.mrm_ecu = VoterState::SUPERVISOR;
  mrm_operation_.comfortable_stop_after_switch = false;
}


void Voter::updateVoterState()
{

  if (mrm_operation_.mrm_ecu == VoterState::UNKNOWN) {
    const auto msg = "invalid mrm_ecu selected in Voter: UNKNOWN";
    throw std::runtime_error(msg);
  }

  if (voter_state_.state == VoterState::NORMAL) {
    if (mrm_operation_.fault_ecu != VoterState::NONE) {
      if (mrm_operation_.mrm_ecu == VoterState::SUPERVISOR) {
        transitionTo(VoterState::SUPERVISOR_STOP);
        voter_state_.fault_ecu = mrm_operation_.fault_ecu;
        voter_state_.comfortable_stop_after_switch = mrm_operation_.comfortable_stop_after_switch;
        return;
      } else if (mrm_operation_.mrm_ecu == VoterState::UNKNOWN) {
        transitionTo(VoterState::SUPERVISOR_STOP);
        voter_state_.fault_ecu = VoterState::UNKNOWN;
        voter_state_.comfortable_stop_after_switch = false;
        return;
      } else if (mrm_operation_.mrm_ecu != VoterState::NONE) {
        // Main or Sub
        if (mrm_operation_.mrm_ecu != convertMrmEcuName(convertSwitchEcu(initial_selected_ecu_))) {
          //cannot directory comfortable stop on extra ecu
          const auto msg = "invalid MRM by Voter: comfortable stop on extra ECU directory";
          throw std::runtime_error(msg);
        }
        transitionTo(VoterState::COMFORTABLE_STOP);
        voter_state_.fault_ecu = mrm_operation_.fault_ecu;
        voter_state_.comfortable_stop_after_switch = false;
        return;
      }
    }
    return;
  }
  if (voter_state_.state == VoterState::SUPERVISOR_STOP) {

    // TODO check if MRC is accomplished
    if (isStopped()) {
      if(voter_state_.comfortable_stop_after_switch) {
        voter_state_.comfortable_stop_after_switch = false;
        if (voter_state_.fault_ecu == VoterState::MAIN) {
          voter_state_.mrm_ecu = VoterState::SUB;
        } else if (voter_state_.fault_ecu == VoterState::SUB) {
          voter_state_.mrm_ecu = VoterState::MAIN;
        } else {
//          RCLCPP_ERROR(this->get_logger(), "Cannot Select MRM ECU when operating comfortable stop after Supervisor Stop: end MRM operation");
          transitionTo(VoterState::MRM_FAILED);
          return;
        }
        transitionTo(VoterState::COMFORTABLE_STOP);
        return;
      }
      transitionTo(VoterState::MRM_SUCCEEDED);
      return;
    }

    if (mrm_operation_.fault_ecu == VoterState::NONE) {
      transitionTo(VoterState::NORMAL);
      voter_state_.fault_ecu = VoterState::NONE;
      voter_state_.mrm_ecu = VoterState::NONE;
      voter_state_.comfortable_stop_after_switch = false;
      return;
    }

    if (mrm_operation_.mrm_ecu == VoterState::NONE) {
      //already switched: do nothing
      return;
    }

    if (mrm_operation_.mrm_ecu != VoterState::SUPERVISOR) {
      // Main or Sub. comfortable stop
      if (mrm_operation_.mrm_ecu != convertMrmEcuName(convertSwitchEcu(initial_selected_ecu_))) {
        //cannot directory comfortable stop on extra ecu
        const auto msg = "invalid MRM by Voter: comfortable stop on extra ECU directory";
        throw std::runtime_error(msg);
      }

      transitionTo(VoterState::COMFORTABLE_STOP);
      voter_state_.fault_ecu = mrm_operation_.fault_ecu;
      voter_state_.mrm_ecu = mrm_operation_.mrm_ecu;
      voter_state_.comfortable_stop_after_switch = false;
      return;
    }

    // if emergency occurs on another single ecu, continue supervisor stop
    voter_state_.fault_ecu = mrm_operation_.fault_ecu;

    // if comfortable_stop_after_switch become false, update it
    if (mrm_operation_.comfortable_stop_after_switch == false) {
      voter_state_.comfortable_stop_after_switch = false;
    }

    return;
  }
  if (voter_state_.state == VoterState::COMFORTABLE_STOP) {
    // TODO check if MRC is accomplished
    if (isStopped()) {
      transitionTo(VoterState::MRM_SUCCEEDED);
      return;
    }

    if (mrm_operation_.fault_ecu == VoterState::NONE) {
      transitionTo(VoterState::NORMAL);
      voter_state_.fault_ecu = VoterState::NONE;
      voter_state_.mrm_ecu = VoterState::NONE;
      voter_state_.comfortable_stop_after_switch = false;
      return;
    }

    if (mrm_operation_.mrm_ecu == VoterState::NONE) {
      //already switched: do nothing
      return;
    }

    if (mrm_operation_.mrm_ecu == VoterState::SUPERVISOR) {
      transitionTo(VoterState::SUPERVISOR_STOP);
      voter_state_.fault_ecu = mrm_operation_.fault_ecu;
      voter_state_.mrm_ecu = mrm_operation_.mrm_ecu;
      voter_state_.comfortable_stop_after_switch = mrm_operation_.comfortable_stop_after_switch;
      return;
    }

    if (mrm_operation_.mrm_ecu != voter_state_.mrm_ecu) {
      if (mrm_operation_.mrm_ecu != convertMrmEcuName(convertSwitchEcu(initial_selected_ecu_))) {
        //cannot directory comfortable stop on extra ecu
        const auto msg = "invalid MRM by Voter: comfortable stop on extra ECU directory";
        throw std::runtime_error(msg);
      }
      voter_state_.mrm_ecu = mrm_operation_.mrm_ecu;
    }


    return;
  }
  if (voter_state_.state == VoterState::MRM_SUCCEEDED) {
    return;  // Do nothing(only checking common recovery events)
  }
  if (voter_state_.state == VoterState::MRM_FAILED) {
      // Do nothing(only checking common recovery events)
    return;
  }
  const auto msg = "invalid state: " + std::to_string(voter_state_.state);
  throw std::runtime_error(msg);
}

bool Voter::isStopped() const
{
  // TODO do not use odom
  constexpr auto th_stopped_velocity = 0.001;
  if (velocity_report_->longitudinal_velocity < th_stopped_velocity) {
    return true;
  }

  return false;
}

bool Voter::isEmergency(const HazardStatusStamped::ConstSharedPtr hazard_status) const
{
  return hazard_status->status.emergency || hazard_status->status.emergency_holding;

}

} // namespace supervisor
