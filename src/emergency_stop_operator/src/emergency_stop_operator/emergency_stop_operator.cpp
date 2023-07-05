#include "emergency_stop_operator/emergency_stop_operator.hpp"

namespace emergency_stop_operator
{

EmergencyStopOperator::EmergencyStopOperator(const rclcpp::NodeOptions & node_options)
: Node("emergency_stop_operator", node_options)
{
  // Parameter
  params_.update_rate = static_cast<int>(declare_parameter<int>("update_rate", 30));
  params_.target_acceleration = declare_parameter<double>("target_acceleration", -2.5);
  params_.target_jerk = declare_parameter<double>("target_jerk", -1.5);
  params_.use_parking_after_stopped = declare_parameter<bool>("use_parking_after_stopped", false);

  // Subscriber
  sub_control_cmd_ = create_subscription<AckermannControlCommand>(
    "~/input/control/control_cmd", 1,
    std::bind(&EmergencyStopOperator::onControlCommand, this, std::placeholders::_1));
  sub_velocity_report_ = create_subscription<VelocityReport>(
    "~/input/vehicle/velocity_report", 1,
    std::bind(&EmergencyStopOperator::onVelocityReport, this, std::placeholders::_1));

  // Server
  service_operation_ = create_service<OperateMrm>(
    "~/input/mrm/emergency_stop/operate", std::bind(
                                            &EmergencyStopOperator::operateEmergencyStop, this,
                                            std::placeholders::_1, std::placeholders::_2));

  // Publisher
  pub_status_ = create_publisher<MrmBehaviorStatus>("~/output/mrm/emergency_stop/status", 1);
  pub_control_cmd_ =
    create_publisher<AckermannControlCommand>("~/output/mrm/emergency_stop/control_cmd", 1);
  pub_gear_cmd_ =
    create_publisher<GearCommand>("~/output/mrm/emergency_stop/gear_cmd", 1);
  pub_hazard_lights_cmd_ =
    create_publisher<HazardLightsCommand>("~/output/mrm/emergency_stop/hazard_lights_cmd", 1);
  pub_turn_indicators_cmd_ =
    create_publisher<TurnIndicatorsCommand>("~/output/mrm/emergency_stop/turn_indicators_cmd", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&EmergencyStopOperator::onTimer, this));

  // Initialize
  status_.state = MrmBehaviorStatus::AVAILABLE;
  is_prev_control_cmd_subscribed_ = false;
}

void EmergencyStopOperator::onControlCommand(AckermannControlCommand::ConstSharedPtr msg)
{
  if (status_.state != MrmBehaviorStatus::OPERATING) {
    prev_control_cmd_ = *msg;
    is_prev_control_cmd_subscribed_ = true;
  }
}

void EmergencyStopOperator::onVelocityReport(VelocityReport::ConstSharedPtr msg)
{
  velocity_report_ = msg;
}

void EmergencyStopOperator::operateEmergencyStop(
  const OperateMrm::Request::SharedPtr request, const OperateMrm::Response::SharedPtr response)
{
  if (request->operate == true) {
    status_.state = MrmBehaviorStatus::OPERATING;
    response->response.success = true;
  } else {
    status_.state = MrmBehaviorStatus::AVAILABLE;
    response->response.success = true;
  }
}

void EmergencyStopOperator::publishStatus() const
{
  auto status = status_;
  status.stamp = this->now();
  pub_status_->publish(status);
}

void EmergencyStopOperator::publishVehicleCommands() const
{
  const auto stamp = this->now();

  GearCommand gear_cmd;
  gear_cmd.stamp = stamp;
  if (params_.use_parking_after_stopped && isStopped()) {
    gear_cmd.command = GearCommand::PARK;
  } else {
    gear_cmd.command = GearCommand::DRIVE;
  }

  HazardLightsCommand hazard_lights_cmd;
  hazard_lights_cmd.stamp = stamp;
  hazard_lights_cmd.command = HazardLightsCommand::ENABLE;

  TurnIndicatorsCommand turn_indicators_cmd;
  turn_indicators_cmd.stamp = stamp;
  turn_indicators_cmd.command = TurnIndicatorsCommand::NO_COMMAND;

  pub_gear_cmd_->publish(gear_cmd);
  pub_hazard_lights_cmd_->publish(hazard_lights_cmd);
  pub_turn_indicators_cmd_->publish(turn_indicators_cmd);
}


void EmergencyStopOperator::publishControlCommand(const AckermannControlCommand & command) const
{
  pub_control_cmd_->publish(command);
}

void EmergencyStopOperator::onTimer()
{
  // TODO not publish by timer
  if (status_.state == MrmBehaviorStatus::OPERATING) {
    auto control_cmd = calcTargetAcceleration(prev_control_cmd_);
    publishControlCommand(control_cmd);
    prev_control_cmd_ = control_cmd;
  } else {
    publishControlCommand(prev_control_cmd_);
  }
  publishStatus();
}

AckermannControlCommand EmergencyStopOperator::calcTargetAcceleration(
  const AckermannControlCommand & prev_control_cmd) const
{
  auto control_cmd = AckermannControlCommand();

  if (!is_prev_control_cmd_subscribed_) {
    control_cmd.stamp = this->now();
    control_cmd.longitudinal.stamp = this->now();
    control_cmd.longitudinal.speed = 0.0;
    control_cmd.longitudinal.acceleration = static_cast<float>(params_.target_acceleration);
    control_cmd.longitudinal.jerk = 0.0;
    control_cmd.lateral.stamp = this->now();
    control_cmd.lateral.steering_tire_angle = 0.0;
    control_cmd.lateral.steering_tire_rotation_rate = 0.0;
    return control_cmd;
  }

  control_cmd = prev_control_cmd;
  const auto dt = (this->now() - prev_control_cmd.stamp).seconds();

  control_cmd.stamp = this->now();
  control_cmd.longitudinal.stamp = this->now();
  control_cmd.longitudinal.speed = static_cast<float>(std::max(
    prev_control_cmd.longitudinal.speed + prev_control_cmd.longitudinal.acceleration * dt, 0.0));
  control_cmd.longitudinal.acceleration = static_cast<float>(std::max(
    prev_control_cmd.longitudinal.acceleration + params_.target_jerk * dt,
    params_.target_acceleration));
  if (prev_control_cmd.longitudinal.acceleration == params_.target_acceleration) {
    control_cmd.longitudinal.jerk = 0.0;
  } else {
    control_cmd.longitudinal.jerk = static_cast<float>(params_.target_jerk);
  }

  return control_cmd;
}

bool EmergencyStopOperator::isStopped() const
{
  constexpr auto th_stopped_velocity = 0.001;
  if (velocity_report_->longitudinal_velocity < th_stopped_velocity) {
    return true;
  }

  return false;
}

}  // namespace emergency_stop_operator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(emergency_stop_operator::EmergencyStopOperator)
