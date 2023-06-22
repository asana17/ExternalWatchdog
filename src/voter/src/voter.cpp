#include "voter/voter.hpp"
#include <tier4_system_msgs/msg/detail/mrm_behavior_status__struct.hpp>
#include <vector>

namespace voter
{

Voter::Voter()
  : Node("voter")
{

  param_.update_rate = declare_parameter<int>("update_rate", 10);
  param_.timeout_hazard_status = declare_parameter<double>("timeout_hazard_status", 0.5);

  // Subscriber
  using std::placeholders::_1;

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&Voter::onOdometry, this, _1));


  // Publisher
  switch_status_pub_ = create_publisher<SwitchStatus>("~/output/switch_status", rclcpp::QoS(500));

  // Initialize
  switch_status_.ecu = SwitchStatus::MAIN;
  switch_status_.initial_ecu = SwitchStatus::MAIN;

  // Initialize each ECUs
  Main_.name = Main;
  Sub_.name = Sub;
  Supervisor_.name = Supervisor;

  for (Ecu* ecu : {&Main_, &Sub_, &Supervisor_}) {
    std::string input_prefix;
    std::string output_prefix;
    std::string external_input_prefix;
    std::string another_external_input_prefix;
    if (ecu->name == Main) {
      input_prefix = "~/input/main";
      output_prefix = "~/output/main";
      external_input_prefix = "~/input/sub";
      another_external_input_prefix = "~/input/supervisor";
      ecu->external_ecu_name = Sub;
      ecu->another_external_ecu_name = Supervisor;
    } else if (ecu->name == Sub) {
      input_prefix = "~/input/sub";
      output_prefix = "~/output/sub";
      external_input_prefix = "~/input/main";
      another_external_input_prefix = "~/input/supervisor";
      ecu->external_ecu_name = Main;
      ecu->another_external_ecu_name = Supervisor;
    } else if (ecu->name == Supervisor) {
      input_prefix = "~/input/supervisor";
      output_prefix = "~/output/supervisor";
      external_input_prefix = "~/input/main";
      another_external_input_prefix = "~/input/sub";
      ecu->external_ecu_name = Main;
      ecu->another_external_ecu_name = Sub;
    }

    self_error_status_[ecu->name] = ErrorStatus{false, false, false};
    external_error_status_[ecu->name] = ErrorStatus{false, false, false};


    // Subscriber
    ecu->sub_self_monitoring_ =
      create_subscription<HazardStatusStamped>(
        input_prefix + "/self_monitoring", rclcpp::QoS{1},
        [ecu, this](const HazardStatusStamped::ConstSharedPtr msg) {
          Voter::onSelfMonitoringStamped(msg, ecu);
        });
    ecu->sub_external_monitoring_ =
      create_subscription<HazardStatusStamped>(
        external_input_prefix + "/self_monitoring", rclcpp::QoS{1},
        [ecu, this](const HazardStatusStamped::ConstSharedPtr msg) {
          Voter::onExternalMonitoringStamped(msg, ecu);
        });
    ecu->sub_another_external_monitoring_ =
      create_subscription<HazardStatusStamped>(
        another_external_input_prefix + "/self_monitoring", rclcpp::QoS{1},
        [ecu, this](const HazardStatusStamped::ConstSharedPtr msg) {
          Voter::onExternalMonitoringStamped(msg, ecu);
        });

    if (ecu->name != Supervisor) {
    ecu->sub_mrm_comfortable_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
      input_prefix + "/mrm/comfortable_stop/status", rclcpp::QoS{1},
        [ecu, this](const MrmBehaviorStatus::ConstSharedPtr msg) {
          Voter::onMrmComfortableStopStatus(msg, ecu);
        });
    }
    ecu->sub_mrm_sudden_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
      input_prefix + "/mrm/emergency_stop/status", rclcpp::QoS{1},
        [ecu, this](const MrmBehaviorStatus::ConstSharedPtr msg) {
          Voter::onMrmSuddenStopStatus(msg, ecu);
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

    // Initialize
    ecu->self_hazard_status_stamped_ = std::make_shared<HazardStatusStamped>();
    ecu->external_hazard_status_stamped_ = std::make_shared<HazardStatusStamped>();
    ecu->another_external_hazard_status_stamped_ = std::make_shared<HazardStatusStamped>();
    ecu->mrm_comfortable_stop_status_ = std::make_shared<MrmBehaviorStatus>();
    ecu->mrm_sudden_stop_status_ = std::make_shared<MrmBehaviorStatus>();

    voter_state_.state = VoterState::NORMAL;
    voter_state_.external_detected = false;
    voter_state_.comfortable_stop_after_switch = false;
    mrm_operation_.fault_ecu = VoterState::NONE;
    mrm_operation_.mrm_ecu = VoterState::NONE;
    mrm_operation_.comfortable_stop_after_switch = false;
  }


  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&Voter::onTimer, this));
}


std::string Voter::convertEcuNameToString(
    ecu_name name) const
{
  if (name == Main) {
    return "Main";
  } else if (name == Sub) {
    return "Sub";
  } else if (name == Supervisor) {
    return "Supervisor";
  } else if (name == None) {
    return "None";
  }
  const auto msg = "invalid ECU name: " + std::to_string(name);
  throw std::runtime_error(msg);
}

void Voter::onSelfMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {

  ecu->self_hazard_status_stamped_ = msg;
  ecu->stamp_self_hazard_status_ = this->now();
  updateSelfErrorStatus();
  judgeMrmOperation();
  updateVoterState();
  operateMrm();
}
void Voter::onExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {

  ecu->external_hazard_status_stamped_ = msg;
  ecu->stamp_external_hazard_status_ = this->now();
  updateExternalErrorStatus();
  judgeMrmOperation();
  updateVoterState();
  operateMrm();
}
void Voter::onAnotherExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, Ecu* ecu) {
  ecu->another_external_hazard_status_stamped_ = msg;
  ecu->stamp_another_external_hazard_status_ = this->now();
  updateExternalErrorStatus();
  judgeMrmOperation();
  updateVoterState();
  operateMrm();
}
void Voter::onMrmComfortableStopStatus(
  const MrmBehaviorStatus::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->mrm_comfortable_stop_status_ = msg;
}

void Voter::onMrmSuddenStopStatus(
  const MrmBehaviorStatus::ConstSharedPtr msg, Ecu* ecu)
{
  ecu->mrm_sudden_stop_status_ = msg;
}

void Voter::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void Voter::onSwitchStatus(const SwitchStatus::ConstSharedPtr msg)
{
  switch_status_ = *msg;
}

void Voter::callMrmBehavior(
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

void Voter::cancelMrmBehavior(
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

bool Voter::isDataReady()
{
  return isEcuDataReady(&Main_) && isEcuDataReady(&Sub_) && isEcuDataReady(&Supervisor_);
}

bool Voter::isEcuDataReady(const Ecu* ecu)
{

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

  return true;
}

void Voter::onTimer()
{
  if (!isDataReady()) {
    return;
  }
  // TODO check hazard_status timeout

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

  RCLCPP_INFO(
    this->get_logger(), "Voter State changed: %s -> %s", state2string(voter_state_.state),
    state2string(new_state));

  voter_state_.state = new_state;
}

void Voter::transitionSwitchTo(const int new_ecu)
{
  const auto ecu2string = [](const int ecu) {
    if (ecu == SwitchStatus::MAIN) {
      return "Main";
    }
    if (ecu == SwitchStatus::SUB) {
      return "Sub";
    }
    if (ecu == SwitchStatus::SUPERVISOR) {
      return "Supervisor";
    }
    const auto msg = "invalid ecu: " + std::to_string(ecu);
    throw std::runtime_error(msg);
  };

  RCLCPP_INFO(
    this->get_logger(), "Switch Status changing: %s -> %s", ecu2string(switch_status_.ecu),
    ecu2string(new_ecu));

  SwitchStatus status;
  status.ecu = new_ecu;
  switch_status_pub_->publish(status);
  switch_status_.ecu = new_ecu;
}


ecu_name Voter::convertSwitchEcu(const SwitchStatus::_ecu_type ecu) const
{

  if (ecu == SwitchStatus::MAIN) {
    return Main;
  } else if (ecu == SwitchStatus::SUB) {
    return Sub;
  } else if (ecu == SwitchStatus::SUPERVISOR) {
    return Supervisor;
  }

  const auto msg = "invalid SwitchStatus: " + std::to_string(ecu);
  throw std::runtime_error(msg);
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


void Voter::updateSelfErrorStatus()
{
  for (Ecu* ecu : {&Main_, &Sub_, &Supervisor_}) {
    const bool is_emergency = ecu->self_hazard_status_stamped_->status.emergency;
    if (is_emergency) {
      const auto switch_ecu = convertSwitchEcu(switch_status_.ecu);

      if (switch_ecu == ecu->name) {
        const bool is_self_recoverable = ecu->self_hazard_status_stamped_->status.self_recoverable;
        // if self_recoverable, emergency_handler already called stop operator
        if (is_self_recoverable) {
          self_error_status_[ecu->name].is_emergency_ = true;
          self_error_status_[ecu->name].stop_operation_needed_ = false;
          self_error_status_[ecu->name].switch_needed_ = false;

        } else {
          //switch and emergency stop in supervisor
          self_error_status_[ecu->name].is_emergency_ = true;
          self_error_status_[ecu->name].stop_operation_needed_ = true;
          self_error_status_[ecu->name].switch_needed_ = true;
        }
      }
      else {
        // operate comfortable stop on switch_ecu
        // non self_recoverable && switch needed fault already occured: already called stop operator
        if (switch_status_.ecu != switch_status_.initial_ecu) {
          // do nothing
          self_error_status_[ecu->name].is_emergency_ = true;
          self_error_status_[ecu->name].stop_operation_needed_ = false;
          self_error_status_[ecu->name].switch_needed_ = false;
        } else {
          // comfortable stop in switch_ecu
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
}

void Voter::updateExternalErrorStatus()
{

  for (Ecu* ecu : {&Main_, &Sub_, &Supervisor_}) {
    const bool is_emergency = checkExternalMonitoring(ecu);
    if (is_emergency) {
      const auto switch_ecu = convertSwitchEcu(switch_status_.ecu);
      if (switch_ecu == ecu->name) {
        //switch and emergency stop in supervisor
        self_error_status_[ecu->name].is_emergency_ = true;
        self_error_status_[ecu->name].is_emergency_ = true;
        self_error_status_[ecu->name].stop_operation_needed_ = true;
        self_error_status_[ecu->name].switch_needed_ = true;
      } else {
        if (switch_status_.ecu != switch_status_.initial_ecu) {
          // do nothing
          self_error_status_[ecu->name].is_emergency_ = true;
          self_error_status_[ecu->name].stop_operation_needed_ = false;
          self_error_status_[ecu->name].switch_needed_ = false;
        } else {
          // comfortable stop in switch_ecu
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
}

bool Voter::checkExternalMonitoring(Ecu * ecu) {
  if (ecu->name == Main) {
    return Sub_.external_hazard_status_stamped_->status.emergency && Supervisor_.external_hazard_status_stamped_->status.emergency;
  } else if (ecu->name == Sub) {
    return Main_.external_hazard_status_stamped_->status.emergency && Supervisor_.external_hazard_status_stamped_->status.emergency;
  } else if (ecu->name == Supervisor) {
    return Main_.external_hazard_status_stamped_->status.emergency && Sub_.external_hazard_status_stamped_->status.emergency;
  }

  const auto msg = "invalid ecu in checkExternalMonitoring: " + std::to_string(ecu->name);
  throw std::runtime_error(msg);
}

void Voter::judgeMrmOperation() {
  getMrmOperationFromExternalMonitoring();
  if (!voter_state_.external_detected) {
    getMrmOperationFromSelfMonitoring();
  }
}

void Voter::getMrmOperationFromSelfMonitoring()
{
  int self_monitoring_result = int(self_error_status_[Main].is_emergency_) + int(self_error_status_[Sub].is_emergency_) + int(self_error_status_[Supervisor].is_emergency_);
  if (self_monitoring_result == 0) {
    getNoMrmOperation();
  } else if (self_monitoring_result == 1) {
    getMrmOperationInternal(self_error_status_);
  } else {
    // self error detection on multiple ECUs
    getMrmOperationMultipleEcuError();
  }
}

void Voter::getMrmOperationFromExternalMonitoring()
{
  int external_monitoring_result = int(external_error_status_[Main].is_emergency_) + int(external_error_status_[Sub].is_emergency_) + int(external_error_status_[Supervisor].is_emergency_);
  if (external_monitoring_result == 0) {
    voter_state_.external_detected = false;
    getNoMrmOperation();
  } else {
    voter_state_.external_detected = true;
    if (external_monitoring_result == 1) {
      getMrmOperationInternal(external_error_status_);
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

void Voter::getMrmOperationInternal(std::map<ecu_name, ErrorStatus> & error_status_)
{
  ecu_name switch_ecu = convertSwitchEcu(switch_status_.ecu);
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
          mrm_operation_.mrm_ecu = convertMrmEcuName(switch_ecu);
          mrm_operation_.comfortable_stop_after_switch = false;
        }
      }
    }
  }
}

void Voter::getMrmOperationMultipleEcuError()
{
  RCLCPP_ERROR(this->get_logger(), "Self error detection on multiple ECUs: operate MRM on Supervisor");
  mrm_operation_.fault_ecu = VoterState::UNKNOWN;
  mrm_operation_.mrm_ecu = VoterState::SUPERVISOR;
  mrm_operation_.comfortable_stop_after_switch = false;
}

void Voter::operateMrm()
{
  if (voter_state_.state == VoterState::NORMAL) {
    if (voter_state_.mrm_ecu != VoterState::NONE) {
      // cancel current MRM
      cancelCurrentMrm();
      voter_state_.mrm_ecu = VoterState::NONE;
      voter_state_.comfortable_stop_after_switch = false;
      transitionSwitchTo(switch_status_.initial_ecu);
    }
    return;
  }
  if (voter_state_.state == VoterState::SUPERVISOR_STOP) {
    voter_state_.fault_ecu = mrm_operation_.fault_ecu;
    voter_state_.comfortable_stop_after_switch = mrm_operation_.comfortable_stop_after_switch;
    if (voter_state_.mrm_ecu != VoterState::SUPERVISOR) {
      cancelCurrentMrm();
      if (switch_status_.ecu != SwitchStatus::SUPERVISOR) {
        switch_status_.ecu = SwitchStatus::SUPERVISOR;
      }
      voter_state_.mrm_ecu = VoterState::SUPERVISOR;
      callMrmBehavior(MrmState::EMERGENCY_STOP, &Supervisor_);
    }
    return;
  }
  if (voter_state_.state == VoterState::COMFORTABLE_STOP) {
    if (voter_state_.mrm_ecu != mrm_operation_.mrm_ecu) {
      cancelCurrentMrm();
      if (mrm_operation_.mrm_ecu == VoterState::MAIN) {
        transitionSwitchTo(SwitchStatus::MAIN);
      } else if (mrm_operation_.mrm_ecu == VoterState::SUB) {
        transitionSwitchTo(SwitchStatus::SUB);
      }
      voter_state_.mrm_ecu = mrm_operation_.mrm_ecu;
    }
  }

}
void Voter::cancelCurrentMrm() {
  for (Ecu* ecu : {&Main_, &Sub_, &Supervisor_}) {
    if (convertMrmEcuName(ecu->name) == voter_state_.mrm_ecu) {
      if (ecu->name == Supervisor) {
        cancelMrmBehavior(MrmState::EMERGENCY_STOP, ecu);
      }
      else {
        cancelMrmBehavior(MrmState::COMFORTABLE_STOP, ecu);
      }
    }
  }
}

void Voter::updateVoterState()
{
  if (voter_state_.state == VoterState::NORMAL) {
    if (mrm_operation_.mrm_ecu == VoterState::SUPERVISOR) {
      transitionTo(VoterState::SUPERVISOR_STOP);
      return;
    } else if (mrm_operation_.mrm_ecu != VoterState::UNKNOWN) {
      transitionTo(VoterState::COMFORTABLE_STOP);
      voter_state_.fault_ecu = mrm_operation_.fault_ecu;
      voter_state_.comfortable_stop_after_switch = mrm_operation_.comfortable_stop_after_switch;
      return;
    }
  } else {
    if (voter_state_.fault_ecu == VoterState::NONE) {
      transitionTo(VoterState::NORMAL);
      voter_state_.fault_ecu = VoterState::NONE;
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
            RCLCPP_ERROR(this->get_logger(), "Cannot Select MRM ECU when operating comfortable stop after Supervisor Stop: end MRM operation");
            transitionTo(VoterState::MRM_FAILED);
          }
          transitionTo(VoterState::COMFORTABLE_STOP);
        }
      }
    } else if (voter_state_.state == VoterState::COMFORTABLE_STOP) {
      // TODO check if MRC is accomplished
      if (isStopped()) {
        transitionTo(VoterState::MRM_SUCCEEDED);
      }
    } else if (voter_state_.state == VoterState::MRM_SUCCEEDED) {
        // Do nothing(only checking common recovery events)
    } else if (voter_state_.state == VoterState::MRM_FAILED) {
      // Do nothing(only checking common recovery events)
    } else {
      const auto msg = "invalid state: " + std::to_string(voter_state_.state);
      throw std::runtime_error(msg);
    }
  }
}

bool Voter::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  if (odom_->twist.twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}

}
