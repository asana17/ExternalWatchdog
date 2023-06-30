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
#include <tier4_system_msgs/msg/detail/mrm_behavior_status__struct.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vector>

namespace supervisor
{

SupervisorNode::SupervisorNode(const rclcpp::NodeOptions & node_options)
  : Node("supervisor", node_options)
{

  param_.update_rate = declare_parameter<int>("update_rate", 10);
  param_.timeout_hazard_status = declare_parameter<double>("timeout_hazard_status", 0.5);

  param_.frame_id = declare_parameter("frame_id", "base_link");

  //Subscriber
  using std::placeholders::_1;

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&SupervisorNode::onOdometry, this, _1));


  CanSwitchInterface_.vehicle_can_sub_ = create_subscription<can_msgs::msg::Frame>("/gsm8/from_can_bus", rclcpp::QoS(500), std::bind(&SupervisorNode::onVehicleCanFrame, this, _1));

  // Publisher
  CanSwitchInterface_.vehicle_can_pub_ = create_publisher<can_msgs::msg::Frame>("/gsm8/to_can_bus", rclcpp::QoS(500));

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
      topic_prefix = "main";
      input_prefix = "~/input/main";
      output_prefix = "~/output/main";
      external_input_prefix = "~/input/sub";
      another_external_input_prefix = "~/input/supervisor";
      ecu->external_ecu_name = Sub;
      ecu->another_external_ecu_name = ecu_name::Supervisor;
    } else if (ecu->name == Sub) {
      topic_prefix = "sub";
      input_prefix = "~/input/sub";
      output_prefix = "~/output/sub";
      external_input_prefix = "~/input/main";
      another_external_input_prefix = "~/input/supervisor";
      ecu->external_ecu_name = Main;
      ecu->another_external_ecu_name = ecu_name::Supervisor;
    } else if (ecu->name == ecu_name::Supervisor) {
      topic_prefix = "supervisor";
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
        external_input_prefix + "/external_monitoring_" + topic_prefix, rclcpp::QoS{1},
        [ecu, this](const HazardStatusStamped::ConstSharedPtr msg) {
          SupervisorNode::onExternalMonitoringStamped(msg, ecu);
        });
    ecu->sub_another_external_monitoring_ =
      create_subscription<HazardStatusStamped>(
        another_external_input_prefix + "/external_monitoring_" + topic_prefix, rclcpp::QoS{1},
        [ecu, this](const HazardStatusStamped::ConstSharedPtr msg) {
          SupervisorNode::onExternalMonitoringStamped(msg, ecu);
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
    ecu->client_mrm_comfortable_stop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ecu->client_mrm_comfortable_stop_ = create_client<OperateMrm>(
        output_prefix + "mrm/comfortable_stop/operate", rmw_qos_profile_services_default,
        ecu->client_mrm_comfortable_stop_group_);
    ecu->client_mrm_sudden_stop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ecu->client_mrm_sudden_stop_ = create_client<OperateMrm>(
        output_prefix + "mrm/comfortable_stop/operate", rmw_qos_profile_services_default,
        ecu->client_mrm_comfortable_stop_group_);

    // Can
    ecu->can_sub_=
      create_subscription<can_msgs::msg::Frame>(
        input_prefix + "/from_can_bus", rclcpp::QoS{500},
        [ecu, this](const can_msgs::msg::Frame::ConstSharedPtr msg) {
          SupervisorNode::onEcuCanFrame(msg, ecu);
        });


    ecu->can_pub_= this->create_publisher<can_msgs::msg::Frame>(
        output_prefix + "/to_can_bus", rclcpp::QoS(500));

    // Initialize
    ecu->self_hazard_status_stamped_ = std::make_shared<HazardStatusStamped>();
    ecu->external_hazard_status_stamped_ = std::make_shared<HazardStatusStamped>();
    ecu->another_external_hazard_status_stamped_ = std::make_shared<HazardStatusStamped>();
    ecu->mrm_comfortable_stop_status_ = std::make_shared<MrmBehaviorStatus>();
    ecu->mrm_sudden_stop_status_ = std::make_shared<MrmBehaviorStatus>();

  }


  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&SupervisorNode::onTimer, this));
}



void SupervisorNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  Voter_.onOdometry(msg);
}


void SupervisorNode::onSelfMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {

  ecu->self_hazard_status_stamped_ = msg;
  ecu->stamp_self_hazard_status_ = this->now();
  const auto selected_ecu = CanSwitchInterface_.getSelectedEcu();
  Voter_.updateSelfErrorStatus(ecu, selected_ecu);
  Voter_.prepareMrmOperation(selected_ecu);
  operateMrm();
}

void SupervisorNode::onExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {

  ecu->external_hazard_status_stamped_ = msg;
  ecu->stamp_external_hazard_status_ = this->now();
  const auto selected_ecu = CanSwitchInterface_.getSelectedEcu();
  Voter_.updateExternalErrorStatus(ecu, selected_ecu);
  Voter_.prepareMrmOperation(selected_ecu);
  operateMrm();
}

void SupervisorNode::onAnotherExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {
  ecu->another_external_hazard_status_stamped_ = msg;
  ecu->stamp_another_external_hazard_status_ = this->now();
  const auto selected_ecu = CanSwitchInterface_.getSelectedEcu();
  Voter_.updateExternalErrorStatus(ecu, selected_ecu);
  Voter_.prepareMrmOperation(selected_ecu);
  operateMrm();
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


void SupervisorNode::onVehicleCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg)
{
  for (auto ecu : {&Main_, &Sub_, &Supervisor_}) {
    CanSwitchInterface_.publishCanToEcu(msg, ecu);
  }
}


// Main or Sub
void SupervisorNode::onEcuCanFrame(const can_msgs::msg::Frame::ConstSharedPtr msg, Ecu* ecu)
{
  if (CanSwitchInterface_.getSelectedEcu() == ecu->name) {
    CanSwitchInterface_.publishCanToVehicle(msg);
  }
}


void SupervisorNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

}


bool SupervisorNode::isDataReady()
{
  return isEcuDataReady();
}

bool SupervisorNode::isEcuDataReady()
{

  for (const auto ecu : {&Main_, &Sub_, &Supervisor_}) {

    const auto ecu_name = convertEcuNameToString(ecu->name);

    if (!ecu->self_hazard_status_stamped_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for self_hazard_status_stamped msg of %s ...", ecu_name.c_str());
      return false;
    }
    if (!ecu->external_hazard_status_stamped_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for external_hazard_status_stamped msg of %s ...", ecu_name.c_str());
    }

    if (
      ecu->name != Supervisor && ecu->mrm_comfortable_stop_status_->state ==
                                       MrmBehaviorStatus::NOT_AVAILABLE) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for mrm comfortable stop to become available on %s ...", ecu_name.c_str());
      return false;
    }

    if (
      ecu->mrm_sudden_stop_status_->state == MrmBehaviorStatus::NOT_AVAILABLE) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "waiting for mrm emergency stop to become available on %s ...", ecu_name.c_str());
      return false;
    }
  }

  return true;
}


void SupervisorNode::callMrmBehavior(
    const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const
{
  const auto ecu_name = convertEcuNameToString(ecu->name);
  auto request = std::make_shared<OperateMrm::Request>();
  request->operate = true;
   if (mrm_behavior == MrmState::COMFORTABLE_STOP) {
    auto result = ecu->client_mrm_comfortable_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Comfortable stop is operated on: %s", ecu_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Comfortable stop is failed to operate on: %s", ecu_name.c_str());
    }
    return;
  }
  if (mrm_behavior == MrmState::EMERGENCY_STOP) {
    auto result = ecu->client_mrm_sudden_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Sudden stop is operated on: %s",ecu_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Sudden stop is failed to operate on: %s", ecu_name.c_str());
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
}

void SupervisorNode::cancelMrmBehavior(
  const MrmState::_behavior_type & mrm_behavior, Ecu* ecu) const
{

  const auto ecu_name = convertEcuNameToString(ecu->name);
  auto request = std::make_shared<OperateMrm::Request>();
  request->operate = false;

  if (mrm_behavior == MrmState::COMFORTABLE_STOP) {
    auto result = ecu->client_mrm_comfortable_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Comfortable stop is canceled on: %s", ecu_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Comfortable stop is failed to cancel on: %s", ecu_name.c_str());
    }
    return;
  }
  if (mrm_behavior == MrmState::EMERGENCY_STOP) {
    auto result = ecu->client_mrm_sudden_stop_->async_send_request(request).get();
    if (result->response.success == true) {
      RCLCPP_WARN(this->get_logger(), "Emergency stop is canceled on: %s", ecu_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Emergency stop is failed to cancel on: %s", ecu_name.c_str());
    }
    return;
  }
  RCLCPP_WARN(this->get_logger(), "invalid MRM behavior: %d", mrm_behavior);
}


void SupervisorNode::operateMrm()
{
  const auto voter_state = Voter_.getVoterState();
  if (voter_state.state == VoterState::NORMAL) {
    if (CurrentMrmStatus_.mrm_ecu != VoterState::NONE) {
      // cancel current MRM
      cancelCurrentMrm();
      CurrentMrmStatus_.mrm_ecu = VoterState::NONE;
      CanSwitchInterface_.changeSwitchTo(initial_selected_ecu_);
    }
    return;
  }
  if (voter_state.state == VoterState::SUPERVISOR_STOP) {
    if (CurrentMrmStatus_.mrm_ecu != VoterState::SUPERVISOR) {
      cancelCurrentMrm();
      CanSwitchInterface_.changeSwitchTo(SwitchStatus::SUPERVISOR);
      CurrentMrmStatus_.mrm_ecu = VoterState::SUPERVISOR;
      callMrmBehavior(MrmState::EMERGENCY_STOP, &Supervisor_);
    }
    return;
  }
  if (voter_state.state == VoterState::COMFORTABLE_STOP) {
    if (CurrentMrmStatus_.mrm_ecu != voter_state.mrm_ecu) {
      cancelCurrentMrm();
      if (voter_state.mrm_ecu == VoterState::MAIN) {
        CanSwitchInterface_.changeSwitchTo(SwitchStatus::MAIN);
      } else if (voter_state.mrm_ecu == VoterState::SUB) {
        CanSwitchInterface_.changeSwitchTo(SwitchStatus::SUB);
      }
      CurrentMrmStatus_.mrm_ecu = voter_state.mrm_ecu;
    }
  }
}

void SupervisorNode::cancelCurrentMrm()
{
  for (Ecu* ecu : {&Main_, &Sub_, &Supervisor_}) {
    if (ecu->name == Supervisor) {
      cancelMrmBehavior(MrmState::EMERGENCY_STOP, ecu);
    }
    else {
      cancelMrmBehavior(MrmState::COMFORTABLE_STOP, ecu);
    }
  }
}

} // namespace supervisor


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(supervisor::SupervisorNode);
