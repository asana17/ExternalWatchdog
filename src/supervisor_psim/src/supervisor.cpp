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

#include "supervisor/supervisor.hpp"
#include "supervisor/ecu.hpp"
#include <rclcpp/callback_group.hpp>
#include <rclcpp/timer.hpp>
#include <tier4_system_msgs/msg/detail/mrm_behavior_status__struct.hpp>
#include <vector>
#include <iostream>
namespace supervisor
{

SupervisorNode::SupervisorNode()
  : Node("supervisor")
{

  param_.update_rate = declare_parameter<int>("update_rate", 30);
  param_.timeout_hazard_status = declare_parameter<double>("timeout_hazard_status", 0.5);
  param_.service_timeout = declare_parameter<double>("service_timeout", 0.01);

  param_.frame_id = declare_parameter("frame_id", "base_link");

  //Subscriber
  using std::placeholders::_1;

  sub_velocity_report_ = create_subscription<VelocityReport>(
    "~/input/velocity_report", rclcpp::QoS{1}, std::bind(&SupervisorNode::onVelocityReport, this, _1));

  // Publisher
  rclcpp::QoS durable_qos{1};
  durable_qos.transient_local();
  ControlSwitchInterface_.vehicle_control_pub_ = create_publisher<AckermannControlCommand>("/control_switch_interface/control_cmd", durable_qos);
  ControlSwitchInterface_.gear_pub_= create_publisher<GearCommand>("/control_switch_interface/gear_cmd", durable_qos);
  ControlSwitchInterface_.turn_indicators_pub_= create_publisher<TurnIndicatorsCommand>("/control_switch_interface/turn_indicators_cmd", durable_qos);
  ControlSwitchInterface_.hazard_lights_pub_= create_publisher<HazardLightsCommand>("/control_switch_interface/hazard_lights_cmd", durable_qos);

  // Initialize each ECUs
  Main_.name = Main;
  Sub_.name = Sub;
  Supervisor_.name = ecu_name::Supervisor;

  for (Ecu* ecu : {&Main_, &Sub_, &Supervisor_}) {
    std::string topic_prefix;
    std::string input_prefix;
    std::string output_prefix;
    std::string external_input_prefix;
    std::string another_external_input_prefix;
    if (ecu->name == Main) {
      topic_prefix = "/main";
      input_prefix = "~/input/main";
      output_prefix = "~/output/main";
      external_input_prefix = "~/input/sub";
      another_external_input_prefix = "~/input/supervisor";
      ecu->external_ecu_name = Sub;
      ecu->another_external_ecu_name = ecu_name::Supervisor;
    } else if (ecu->name == Sub) {
      topic_prefix = "/sub";
      input_prefix = "~/input/sub";
      output_prefix = "~/output/sub";
      external_input_prefix = "~/input/main";
      another_external_input_prefix = "~/input/supervisor";
      ecu->external_ecu_name = Main;
      ecu->another_external_ecu_name = ecu_name::Supervisor;
    } else if (ecu->name == ecu_name::Supervisor) {
      topic_prefix = "/supervisor";
      input_prefix = "~/input/supervisor";
      output_prefix = "~/output/supervisor";
      external_input_prefix = "~/input/main";
      another_external_input_prefix = "~/input/sub";
      ecu->external_ecu_name = Main;
      ecu->another_external_ecu_name = Sub;
    }



    // Subscriber
    ecu->sub_self_monitoring_ =
      create_subscription<HazardStatusStamped>(
        input_prefix + "/self_monitoring", rclcpp::QoS{1},
        [ecu, this](const HazardStatusStamped::ConstSharedPtr msg) {
        SupervisorNode::onSelfMonitoringStamped(msg, ecu);
        });

    ecu->sub_external_monitoring_ =
      create_subscription<HazardStatusStamped>(
        external_input_prefix + "/external_monitoring" + topic_prefix, rclcpp::QoS{1},
        [ecu, this](const HazardStatusStamped::ConstSharedPtr msg) {
          SupervisorNode::onExternalMonitoringStamped(msg, ecu);
        });

    ecu->sub_another_external_monitoring_ =
      create_subscription<HazardStatusStamped>(
        another_external_input_prefix + "/external_monitoring" + topic_prefix, rclcpp::QoS{1},
        [ecu, this](const HazardStatusStamped::ConstSharedPtr msg) {
          SupervisorNode::onAnotherExternalMonitoringStamped(msg, ecu);
        });


    if (ecu->name != ecu_name::Supervisor) {
    ecu->sub_mrm_comfortable_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
      input_prefix + "/mrm/comfortable_stop/status", rclcpp::QoS{1},
        [ecu, this](const MrmBehaviorStatus::ConstSharedPtr msg) {
          SupervisorNode::onMrmComfortableStopStatus(msg, ecu);
        });
    }
    ecu->sub_mrm_sudden_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
      input_prefix + "/mrm/emergency_stop/status", rclcpp::QoS{1},
        [ecu, this](const MrmBehaviorStatus::ConstSharedPtr msg) {
          SupervisorNode::onMrmSuddenStopStatus(msg, ecu);
        });


    // Client
    if (ecu->name != ecu_name::Supervisor) {
      ecu->client_mrm_comfortable_stop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      ecu->client_mrm_comfortable_stop_ = create_client<OperateMrm>(
          output_prefix + "/mrm/comfortable_stop/operate", rmw_qos_profile_services_default,
          ecu->client_mrm_comfortable_stop_group_);
    }
    ecu->client_mrm_sudden_stop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ecu->client_mrm_sudden_stop_ = create_client<OperateMrm>(
        output_prefix + "/mrm/emergency_stop/operate", rmw_qos_profile_services_default,
        ecu->client_mrm_comfortable_stop_group_);

    // Control

    ecu->control_sub_ =
      create_subscription<AckermannControlCommand>(
        input_prefix + "/control_cmd", 1,
        [ecu, this](const AckermannControlCommand::ConstSharedPtr msg) {
          SupervisorNode::onEcuControlCmd(msg, ecu);
        });
    ecu->gear_sub_ =
      create_subscription<GearCommand>(
        input_prefix + "/gear_cmd", 1,
        [ecu, this](const GearCommand::ConstSharedPtr msg) {
          SupervisorNode::onEcuGearCmd(msg, ecu);
        });
    ecu->turn_indicators_sub_=
      create_subscription<TurnIndicatorsCommand>(
        input_prefix + "/turn_indicators_cmd", 1,
        [ecu, this](const TurnIndicatorsCommand::ConstSharedPtr msg) {
          SupervisorNode::onEcuTurnIndicatorsCmd(msg, ecu);
        });
    ecu->hazard_lights_sub_=
      create_subscription<HazardLightsCommand>(
        input_prefix + "/hazard_lights_cmd", 1,
        [ecu, this](const HazardLightsCommand::ConstSharedPtr msg) {
          SupervisorNode::onEcuHazardLightsCmd(msg, ecu);
        });


    // Initialize
    ecu->mrm_comfortable_stop_status_ = std::make_shared<MrmBehaviorStatus>();
    ecu->mrm_sudden_stop_status_ = std::make_shared<MrmBehaviorStatus>();
    CurrentMrmStatus_.mrm_ecu = VoterState::NONE;

  }

  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&SupervisorNode::onTimer, this));
}

void SupervisorNode::onVelocityReport(const VelocityReport::ConstSharedPtr msg)
{
  Voter_.onVelocityReport(msg);
}

void SupervisorNode::onSelfMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {

  ecu->self_hazard_status_stamped_ = msg;
  ecu->stamp_self_hazard_status_ = this->now();
}

void SupervisorNode::onExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {
  ecu->external_hazard_status_stamped_ = msg;
  ecu->stamp_external_hazard_status_ = this->now();
}

void SupervisorNode::onAnotherExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {
  ecu->another_external_hazard_status_stamped_ = msg;
  ecu->stamp_another_external_hazard_status_ = this->now();
}

void SupervisorNode::onMrmComfortableStopStatus(
  const MrmBehaviorStatus::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->mrm_comfortable_stop_status_ = msg;
}

void SupervisorNode::onMrmSuddenStopStatus(
  const MrmBehaviorStatus::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->mrm_sudden_stop_status_ = msg;
}


void SupervisorNode::onEcuControlCmd(const AckermannControlCommand::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->control_cmd_ = msg;
}

void SupervisorNode::onEcuGearCmd(const GearCommand::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->gear_cmd_ = msg;
}

void SupervisorNode::onEcuTurnIndicatorsCmd(const TurnIndicatorsCommand::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->turn_indicators_cmd_ = msg;
}

void SupervisorNode::onEcuHazardLightsCmd(const HazardLightsCommand::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->hazard_lights_cmd_ = msg;
}

void SupervisorNode::onTimer()
{

  if (!is_data_ready_ && !isDataReady()) {
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "data not ready for supervisor");
    return;
  }
  is_data_ready_ = true;

  publishControlCommands();

  switch_selected_ecu_ = ControlSwitchInterface_.getSelectedEcu();

  for (const auto ecu: {&Main_, &Sub_, &Supervisor_}) {
    Voter_.updateSelfErrorStatus(ecu, switch_selected_ecu_);
    Voter_.updateExternalErrorStatus(ecu, switch_selected_ecu_);
  }

  Voter_.prepareMrmOperation(switch_selected_ecu_);
  operateMrm();
}

void SupervisorNode::publishControlCommands()
{

  switch_selected_ecu_ = ControlSwitchInterface_.getSelectedEcu();
  RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "%s is selected...", convertEcuNameToString(switch_selected_ecu_).c_str());

  for (const auto ecu: {&Main_, &Sub_, &Supervisor_}) {
    if (ecu->name == switch_selected_ecu_) {
      ControlSwitchInterface_.publishControlToVehicle(ecu->control_cmd_);
      ControlSwitchInterface_.publishGearToVehicle(ecu->gear_cmd_);
      ControlSwitchInterface_.publishTurnIndicatorsToVehicle(ecu->turn_indicators_cmd_);
      ControlSwitchInterface_.publishHazardLightsToVehicle(ecu->hazard_lights_cmd_);
    }
  }
}


bool SupervisorNode::isDataReady()
{
  return isEcuDataReady() && isControlDataReady();
}

bool SupervisorNode::isEcuDataReady()
{
  bool is_data_ready = true;
  for (const auto ecu : {&Main_, &Sub_, &Supervisor_}) {
    const auto ecu_name = convertEcuNameToString(ecu->name);
    if (!ecu->self_hazard_status_stamped_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for self_hazard_status_stamped msg of %s ...", ecu_name.c_str());
      is_data_ready = false;
    }
    if (!ecu->external_hazard_status_stamped_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for external_hazard_status_stamped msg of %s ...", ecu_name.c_str());
      is_data_ready = false;
    }
    if (!ecu->another_external_hazard_status_stamped_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for another_external_hazard_status_stamped msg of %s ...", ecu_name.c_str());
      is_data_ready = false;
    }
    // temporarily checking Main only (without sub running)
    if (
      ecu->name != Supervisor && ecu->mrm_comfortable_stop_status_->state ==
                                       MrmBehaviorStatus::NOT_AVAILABLE) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for mrm comfortable stop to become available on %s ...", ecu_name.c_str());
      is_data_ready = false;
    }
    if (ecu->mrm_sudden_stop_status_->state == MrmBehaviorStatus::NOT_AVAILABLE) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for mrm emergency stop to become available on %s ...", ecu_name.c_str());
      is_data_ready = false;
    }
  }

  return is_data_ready;
}

bool SupervisorNode::isControlDataReady()
{
  bool is_data_ready = true;
  for (const auto ecu: {&Main_, &Sub_, &Supervisor_}) {
    if (ecu->name == ControlSwitchInterface_.getSelectedEcu()) {
      const auto ecu_name = convertEcuNameToString(ecu->name);
      if (!ecu->control_cmd_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
          "waiting for control_cmd to become available on %s ...", ecu_name.c_str());
        is_data_ready = false;
      }
      if (!ecu->gear_cmd_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
          "waiting for gear_cmd to become available on %s ...", ecu_name.c_str());
        is_data_ready = false;
      }
      if (!ecu->turn_indicators_cmd_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
          "waiting for turn_indicators_cmd_to become available on %s ...", ecu_name.c_str());
        is_data_ready = false;
      }
      if (!ecu->hazard_lights_cmd_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
          "waiting for hazard_lights_cmd_ to become available on %s ...", ecu_name.c_str());
        is_data_ready = false;
      }
    }
  }
  return is_data_ready;
}


void SupervisorNode::callMrmBehavior(
    const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const
{
  auto request = std::make_shared<OperateMrm::Request>();
  request->operate = true;

  service_result result = No_Response;
  std::string mrm_behavior_str;

  if (mrm_behavior == MrmState::COMFORTABLE_STOP) {
    mrm_behavior_str = "comfortable_stop";
    result = callMrmService(request, ecu->client_mrm_comfortable_stop_);
  }
  if (mrm_behavior == MrmState::EMERGENCY_STOP) {
    std::cout << "calling emergency stop on: " << convertEcuNameToString(ecu->name) << std::endl;
    mrm_behavior_str = "emergency_stop";
    result = callMrmService(request, ecu->client_mrm_sudden_stop_);
  }
  const auto ecu_name = convertEcuNameToString(ecu->name);
  if (result == Success) {
    RCLCPP_WARN(this->get_logger(), "%s stop is called on: %s", mrm_behavior_str.c_str(), ecu_name.c_str());
  } else if (result == Failure) {
    RCLCPP_ERROR(this->get_logger(), "%s is failed to call on: %s", mrm_behavior_str.c_str(), ecu_name.c_str());
  } else if (result == Timeout) {
      RCLCPP_ERROR(this->get_logger(), "MRM Service did not respond on: %s", ecu_name.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
  }
}

void SupervisorNode::cancelMrmBehavior(
  const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const
{

  auto request = std::make_shared<OperateMrm::Request>();
  request->operate = false;

  service_result result = No_Response;
  std::string mrm_behavior_str;

  if (mrm_behavior == MrmState::COMFORTABLE_STOP) {
    mrm_behavior_str = "comfortable_stop";
    result = callMrmService(request, ecu->client_mrm_comfortable_stop_);
  }
  if (mrm_behavior == MrmState::EMERGENCY_STOP) {
    mrm_behavior_str = "emergency_stop";
    result = callMrmService(request, ecu->client_mrm_sudden_stop_);
  }

  const auto ecu_name = convertEcuNameToString(ecu->name);
  if (result == Success) {
    RCLCPP_WARN(this->get_logger(), "%s stop is canceled on: %s", mrm_behavior_str.c_str(), ecu_name.c_str());
  } else if (result == Failure) {
    RCLCPP_ERROR(this->get_logger(), "%s is failed to cancel on: %s", mrm_behavior_str.c_str(), ecu_name.c_str());
  } else if (result == Timeout) {
      RCLCPP_ERROR(this->get_logger(), "MRM Service did not respond on: %s", ecu_name.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
  }
}

service_result SupervisorNode::callMrmService(
    const OperateMrm::Request::SharedPtr request, rclcpp::Client<OperateMrm>::SharedPtr mrm_client) const
{
  const auto duration = std::chrono::duration<double, std::ratio<1>>(param_.service_timeout);
  auto future = mrm_client->async_send_request(request);
  if (future.wait_for(duration) != std::future_status::ready) {
      return Timeout;
  }
  auto result = future.get();
  if (result->response.success) {
    return Success;
  }
  return Failure;
}


void SupervisorNode::operateMrm()
{
  const auto voter_state = Voter_.getVoterState();
  if (voter_state.state == VoterState::NORMAL) {
    if (CurrentMrmStatus_.mrm_ecu != VoterState::NONE) {
      ControlSwitchInterface_.changeSwitchTo(initial_selected_ecu_);
      // cancel current MRM
      if (checkMrmStatus(CurrentMrmStatus_.mrm_ecu) != MrmBehaviorStatus::AVAILABLE) {
        cancelCurrentMrm(CurrentMrmStatus_.mrm_ecu);
        return;
      }
      CurrentMrmStatus_.mrm_ecu = VoterState::NONE;
    }
    return;
  }
  if (voter_state.state == VoterState::SUPERVISOR_STOP) {
    std::cout << "operateMrm SUPERVISOR_STOP" << std::endl;
    if (CurrentMrmStatus_.mrm_ecu != VoterState::SUPERVISOR) {
      bool appropriate_mrm_operating = true;
      if (checkMrmStatus(CurrentMrmStatus_.mrm_ecu) == MrmBehaviorStatus::OPERATING) {
        appropriate_mrm_operating = false;
        cancelCurrentMrm(CurrentMrmStatus_.mrm_ecu);
      }
      if (checkMrmStatus(VoterState::SUPERVISOR) != MrmBehaviorStatus::OPERATING) {
        appropriate_mrm_operating = false;
        callMrmBehavior(MrmState::EMERGENCY_STOP, &Supervisor_);
      }
      ControlSwitchInterface_.changeSwitchTo(SwitchStatus::SUPERVISOR);
      if (appropriate_mrm_operating) {
        CurrentMrmStatus_.mrm_ecu = VoterState::SUPERVISOR;
      }
    }
    return;
  }
  if (voter_state.state == VoterState::COMFORTABLE_STOP) {
    std::cout << "operateMrm COMFORTABLE_STOP" << std::endl;
    if (CurrentMrmStatus_.mrm_ecu != voter_state.mrm_ecu) {
      bool appropriate_mrm_operating = true;
      if (checkMrmStatus(CurrentMrmStatus_.mrm_ecu) == MrmBehaviorStatus::OPERATING) {
        appropriate_mrm_operating = false;
        cancelCurrentMrm(CurrentMrmStatus_.mrm_ecu);
      }
      if (voter_state.mrm_ecu == VoterState::MAIN) {
        if(checkMrmStatus(VoterState::MAIN) != MrmBehaviorStatus::OPERATING) {
          appropriate_mrm_operating = false;
          callMrmBehavior(MrmState::COMFORTABLE_STOP, &Main_);
          std::cout << "MRM called on Main" << std::endl;
        }
        ControlSwitchInterface_.changeSwitchTo(SwitchStatus::MAIN);
        if (appropriate_mrm_operating) {
          CurrentMrmStatus_.mrm_ecu = VoterState::MAIN;
        }
        return;
      }
      if (voter_state.mrm_ecu == VoterState::SUB) {
        if(checkMrmStatus(VoterState::SUB) != MrmBehaviorStatus::OPERATING) {
          appropriate_mrm_operating = false;
          callMrmBehavior(MrmState::COMFORTABLE_STOP, &Sub_);
          std::cout << "MRM called on Sub" << std::endl;
        }
        ControlSwitchInterface_.changeSwitchTo(SwitchStatus::SUB);
        if (appropriate_mrm_operating) {
          CurrentMrmStatus_.mrm_ecu = VoterState::SUB;
        }
      }
    }
  }
}

void SupervisorNode::cancelCurrentMrm(const VoterState::_mrm_ecu_type & mrm_ecu)
{
  // TODO currently only comfortable stop is called for Main & Sub MRM
  for (const auto ecu: {&Main_, &Sub_, &Supervisor_}) {
    if (ecu->name == convertMrmEcu(mrm_ecu)) {
      if (ecu->name == Supervisor) {
        cancelMrmBehavior(MrmState::EMERGENCY_STOP, ecu);
      }
      else {
        cancelMrmBehavior(MrmState::COMFORTABLE_STOP, ecu);
      }
    }
  }
  // no mrm operator to cancel
  return;
}

MrmBehaviorStatus::_state_type SupervisorNode::checkMrmStatus(const VoterState::_mrm_ecu_type & mrm_ecu) const
{
  // TODO currently only comfortable stop is called for Main & Sub MRM
  for (const auto ecu: {&Main_, &Sub_, &Supervisor_}) {
    if (ecu->name == convertMrmEcu(mrm_ecu)) {
      if (ecu->name == Supervisor) {
        return ecu->mrm_sudden_stop_status_->state;
      }
      else {
        return ecu->mrm_comfortable_stop_status_->state;
      }
    }
  }
  return MrmBehaviorStatus::NOT_AVAILABLE;
}


} // namespace supervisor
