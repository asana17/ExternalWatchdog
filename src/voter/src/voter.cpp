#include "voter/voter.hpp"

namespace voter
{

Voter::Voter()
  : Node("voter")
{

  param_.update_rate = declare_parameter<int>("update_rate", 10);

  // initialize each ECUs
  Main_.name = ecu_name::Main;
  Sub_.name = ecu_name::Sub;
  Supervisor_.name = ecu_name::Supervisor;

  for (ECU* ecu : {&Main_, &Sub_, &Supervisor_}) {
    std::string input_prefix;
    std::string output_prefix;
    if (ecu->name == ecu_name::Main) {
      input_prefix = "~/input/main";
      output_prefix = "~/output/main";
    } else if (ecu->name == ecu_name::Sub) {
      input_prefix = "~/input/sub";
      output_prefix = "~/output/sub";
    } else if (ecu->name == ecu_name::Supervisor) {
      input_prefix = "~/input/supervisor";
      output_prefix = "~/output/supervisor";
    }

    // Subscriber
    ecu->sub_self_monitoring_ =
      create_subscription<HazardStatusStamped>(
        input_prefix + "/self_monitoring", rclcpp::QoS{1},
        [ecu](const HazardStatusStamped::ConstSharedPtr msg) {
          Voter::onSelfMonitoringStamped(msg, ecu);
        });
    ecu->sub_external_monitoring_ =
      create_subscription<HazardStatusStamped>(
        input_prefix + "/self_monitoring", rclcpp::QoS{1},
        [ecu](const HazardStatusStamped::ConstSharedPtr msg) {
          Voter::onExternalMonitoringStamped(msg, ecu);
        });
    if (ecu->name != ecu_name::Supervisor) {
    ecu->sub_mrm_comfortable_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
      input_prefix + "/mrm/comfortable_stop/status", rclcpp::QoS{1},
        [ecu](const MrmBehaviorStatus::ConstSharedPtr msg) {
          Voter::onMrmComfortableStopStatus(msg, ecu);
        });
    }
    ecu->sub_mrm_sudden_stop_status_ = create_subscription<tier4_system_msgs::msg::MrmBehaviorStatus>(
      input_prefix + "/mrm/emergency_stop/status", rclcpp::QoS{1},
        [ecu](const MrmBehaviorStatus::ConstSharedPtr msg) {
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
  }


  // Timer
  const auto update_period_ns = rclcpp::Rate(param_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&Voter::onTimer, this));
}


void Voter::onSelfMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, ECU* ecu) {

  ecu->self_hazard_status_stamped_ = msg;
}
void Voter::onExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, ECU* ecu) {

  ecu->external_hazard_status_stamped_ = msg;
}
void Voter::onMrmComfortableStopStatus(
  const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg, ECU* ecu)
{
  ecu->mrm_comfortable_stop_status_ = msg;
}

void Voter::onMrmSuddenStopStatus(
  const tier4_system_msgs::msg::MrmBehaviorStatus::ConstSharedPtr msg, ECU* ecu)
{
  ecu->mrm_sudden_stop_status_ = msg;
}


bool Voter::isDataReady()
{
  //TODO
  return isEcuDataReady(&Main_) && isEcuDataReady(&Sub_) && isEcuDataReady(&Supervisor_);
}

bool Voter::isEcuDataReady(const ECU* ecu)
{

  if (!ecu->self_hazard_status_stamped_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for self_hazard_status_stamped msg...");
    return false;
  }
  if (!ecu->external_hazard_status_stamped_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for external_hazard_status_stamped msg...");
    return false;
  }

  if (
    ecu->name != ecu_name::Supervisor && ecu->mrm_comfortable_stop_status_->state ==
                                     tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for mrm comfortable stop to become available...");
    return false;
  }

  if (
    ecu->mrm_sudden_stop_status_->state == tier4_system_msgs::msg::MrmBehaviorStatus::NOT_AVAILABLE) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for mrm emergency stop to become available...");
    return false;
  }

  return true;
}

void Voter::onTimer()
{
  if (!isDataReady()) {
    return;
  }
}


}
