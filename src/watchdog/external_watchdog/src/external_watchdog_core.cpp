// Copyright 2022 Tier IV, Inc.
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

#include "external_watchdog/external_watchdog_core.hpp"


ExternalWatchdog::ExternalWatchdog()
: Node(
    "external_watchdog", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
  vehicle_info_(vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo())
{
  // Parameters
  base_frame_id_ = declare_parameter("base_frame_id", "base_link");
  get_parameter_or<int>("update_rate", params_.update_rate, 10);
  get_parameter_or<double>("data_timeout", params_.data_timeout, 0.10);

  /* parameters for vehicle specifications */
  tire_radius_ = vehicle_info_.wheel_radius_m;
  wheel_base_ = vehicle_info_.wheel_base_m;

  steering_offset_ = declare_parameter("steering_offset", 0.0);
  enable_steering_rate_control_ = declare_parameter("enable_steering_rate_control", false);

  /* parameters for emergency stop */
  emergency_brake_ = declare_parameter("emergency_brake", 0.7);
  
  /* vehicle parameters */
  vgr_coef_a_ = declare_parameter("vgr_coef_a", 15.713);
  vgr_coef_b_ = declare_parameter("vgr_coef_b", 0.053);
  vgr_coef_c_ = declare_parameter("vgr_coef_c", 0.042);
  accel_pedal_offset_ = declare_parameter("accel_pedal_offset", 0.0);
  brake_pedal_offset_ = declare_parameter("brake_pedal_offset", 0.0);
  speed_scale_factor_ = declare_parameter("speed_scale_factor", 1.0);

  /* parameters for limitter */
  max_throttle_ = declare_parameter("max_throttle", 0.2);
  max_brake_ = declare_parameter("max_brake", 0.8);
  max_steering_wheel_ = declare_parameter("max_steering_wheel", 2.7 * M_PI);
  max_steering_wheel_rate_ = declare_parameter("max_steering_wheel_rate", 6.6);
  min_steering_wheel_rate_ = declare_parameter("min_steering_wheel_rate", 0.5);
  steering_wheel_rate_low_vel_ = declare_parameter("steering_wheel_rate_low_vel", 5.0);
  steering_wheel_rate_stopped_ = declare_parameter("steering_wheel_rate_stopped", 5.0);
  low_vel_thresh_ = declare_parameter("low_vel_thresh", 1.389);  // 5.0kmh


  using std::placeholders::_1;

  // Subscriber
  sub_hazard_status_ = create_subscription<watchdog_system_msgs::msg::HazardStatusStamped>(
    "input/hazard_status", rclcpp::QoS{1}, std::bind(&ExternalWatchdog::onHazardStatus, this, _1));

  sub_tilde_hazard_status_ = create_subscription<watchdog_system_msgs::msg::TildeHazardStatusStamped>(
    "input/tilde_hazard_status", rclcpp::QoS{1}, std::bind(&ExternalWatchdog::onTildeHazardStatus, this, _1));

 sub_prev_control_command_ =
    create_subscription<watchdog_system_msgs::msg::AckermannControlCommand>(
      "~/input/prev_control_command", rclcpp::QoS{1},
      std::bind(&ExternalWatchdog::onPrevControlCommand, this, _1));

 sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{1}, std::bind(&ExternalWatchdog::onOdometry, this, _1));


  // From pacmod

  steer_wheel_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
      this, "/pacmod/steering_rpt");
  wheel_speed_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>(
      this, "/pacmod/wheel_speed_rpt");
  accel_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/accel_rpt");
  brake_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>(
    this, "/pacmod/brake_rpt");
  shift_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
    this, "/pacmod/shift_rpt");
  turn_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
    this, "/pacmod/turn_rpt");
  global_rpt_sub_ = std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>>(
    this, "/pacmod/global_rpt");
  rear_door_rpt_sub_ =
    std::make_unique<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>>(
      this, "/pacmod/rear_pass_door_rpt");

 pacmod_feedbacks_sync_ =
    std::make_unique<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>>(
      PacmodFeedbacksSyncPolicy(10), *steer_wheel_rpt_sub_, *wheel_speed_rpt_sub_, *accel_rpt_sub_,
      *brake_rpt_sub_, *shift_rpt_sub_, *turn_rpt_sub_, *global_rpt_sub_, *rear_door_rpt_sub_);

  pacmod_feedbacks_sync_->registerCallback(std::bind(
    &ExternalWatchdog::callbackPacmodRpt, this, std::placeholders::_1, std::placeholders::_2,
    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6,
    std::placeholders::_7, std::placeholders::_8));

 // To pacmod
  accel_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/accel_cmd", rclcpp::QoS{1});
  brake_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdFloat>("/pacmod/brake_cmd", rclcpp::QoS{1});
  steer_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SteeringCmd>("/pacmod/steering_cmd", rclcpp::QoS{1});
  shift_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/shift_cmd", rclcpp::QoS{1});
  turn_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/turn_cmd", rclcpp::QoS{1});
  door_cmd_pub_ =
    create_publisher<pacmod3_msgs::msg::SystemCmdInt>("/pacmod/rear_pass_door_cmd", rclcpp::QoS{1});
  raw_steer_cmd_pub_ = create_publisher<pacmod3_msgs::msg::SteeringCmd>(
    "/pacmod/raw_steer_cmd", rclcpp::QoS{1});  // only for debug

  const auto period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ExternalWatchdog::onTimer, this));
}


void ExternalWatchdog::onHazardStatus(const watchdog_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg)
{
  hazard_status_ = msg;
  hazard_status_sub_time_ = this->now();
  const auto hazard_status_level = hazard_status_->status.level;
  judgeHazardStatus(hazard_status_level);
}

void ExternalWatchdog::onTildeHazardStatus(const watchdog_system_msgs::msg::TildeHazardStatusStamped::ConstSharedPtr msg)
{
  tilde_hazard_status_ = msg;
  tilde_hazard_status_sub_time_ = this->now();
  const auto tilde_hazard_status_level = tilde_hazard_status_->status.level;
  judgeHazardStatus(tilde_hazard_status_level);

}


void ExternalWatchdog::onPrevControlCommand(
  const watchdog_system_msgs::msg::AckermannControlCommand::ConstSharedPtr msg)
{
  auto control_command = new watchdog_system_msgs::msg::AckermannControlCommand(*msg);
  control_command->stamp = msg->stamp;
  control_cmd_ptr_ =
    watchdog_system_msgs::msg::AckermannControlCommand::ConstSharedPtr(control_command);
}


void ExternalWatchdog::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
}

void ExternalWatchdog::onTimer()
{
  //check timeout
  const auto now_time = this->now();
  if (((now_time - hazard_status_sub_time_).seconds() > params_.data_timeout) || (((now_time - tilde_hazard_status_sub_time_).seconds() > params_.data_timeout ))) {
    callDirectMRM();
  }
}

#define DIRECT_MRM_LEVEL 2

void ExternalWatchdog::judgeHazardStatus(const uint8_t hazard_status_level)
{
  if (hazard_status_level >= DIRECT_MRM_LEVEL) {
    callDirectMRM();
  }
}

void ExternalWatchdog::callDirectMRM()
{
  //TODO
  engage_cmd_ = true;
  publishPacmodCommands();
}


bool ExternalWatchdog::isStopped()
{
  constexpr auto th_stopped_velocity = 0.001;
  if (odom_->twist.twist.linear.x < th_stopped_velocity) {
    return true;
  }

  return false;
}

/*watchdog_system_msgs::msg::AckermannControlCommand ExternalWatchdog::getEmergencyControlCommand()
{
  watchdog_system_msgs::msg::AckermannControlCommand emergency_stop_cmd;
  emergency_stop_cmd.lateral = control_cmd_ptr_->lateral;
  emergency_stop_cmd.longitudinal.speed = 0.0;
  emergency_stop_cmd.longitudinal.acceleration = -2.5;

  return emergency_stop_cmd;
}*/

/* check pacmod enabled */

void ExternalWatchdog::callbackPacmodRpt(
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
  const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
  const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr shift_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
  const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt,
  const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr rear_door_rpt)
{
  is_pacmod_rpt_received_ = true;
  steer_wheel_rpt_ptr_ = steer_wheel_rpt;
  wheel_speed_rpt_ptr_ = wheel_speed_rpt;
  accel_rpt_ptr_ = accel_rpt;
  brake_rpt_ptr_ = brake_rpt;
  gear_cmd_rpt_ptr_ = shift_rpt;
  global_rpt_ptr_ = global_rpt;
  turn_rpt_ptr_ = turn_rpt;

  is_pacmod_enabled_ =
    steer_wheel_rpt_ptr_->enabled && accel_rpt_ptr_->enabled && brake_rpt_ptr_->enabled;
  RCLCPP_DEBUG(
    get_logger(),
    "enabled: is_pacmod_enabled_ %d, steer %d, accel %d, brake %d, shift %d, "
    "global %d",
    is_pacmod_enabled_, steer_wheel_rpt_ptr_->enabled, accel_rpt_ptr_->enabled,
    brake_rpt_ptr_->enabled, gear_cmd_rpt_ptr_->enabled, global_rpt_ptr_->enabled);
}

void ExternalWatchdog::publishPacmodCommands()
{

  const rclcpp::Time current_time = get_clock()->now();

  double desired_throttle = 0.0;
  double desired_brake = emergency_brake_;

  const double current_velocity =
    calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *gear_cmd_rpt_ptr_);
  const double current_steer_wheel = steer_wheel_rpt_ptr_->output;
  const double adaptive_gear_ratio =
    calculateVariableGearRatio(current_velocity, current_steer_wheel);
  double desired_steer_wheel =
    (control_cmd_ptr_->lateral.steering_tire_angle - steering_offset_) * adaptive_gear_ratio;
  desired_steer_wheel =
    std::min(std::max(desired_steer_wheel, -max_steering_wheel_), max_steering_wheel_);
  uint16_t desired_shift = gear_cmd_rpt_ptr_->output;
  //TODO check shift change

  /* check clear flag */
  bool clear_override = false;
  if (is_pacmod_enabled_ == true) {
    is_clear_override_needed_ = false;
  } else {
    clear_override = true;
  }

  /* publish accel cmd */
  {
    pacmod3_msgs::msg::SystemCmdFloat accel_cmd;
    accel_cmd.header.frame_id = base_frame_id_;
    accel_cmd.header.stamp = current_time;
    accel_cmd.enable = engage_cmd_;
    accel_cmd.ignore_overrides = false;
    accel_cmd.clear_override = clear_override;
    accel_cmd.command = std::max(0.0, std::min(desired_throttle, max_throttle_));
    accel_cmd_pub_->publish(accel_cmd);
  }

  /* publish brake cmd */
  {
    pacmod3_msgs::msg::SystemCmdFloat brake_cmd;
    brake_cmd.header.frame_id = base_frame_id_;
    brake_cmd.header.stamp = current_time;
    brake_cmd.enable = engage_cmd_;
    brake_cmd.ignore_overrides = false;
    brake_cmd.clear_override = clear_override;
    brake_cmd.command = std::max(0.0, std::min(desired_brake, max_brake_));
    brake_cmd_pub_->publish(brake_cmd);
  }

  /* publish steering cmd */
  {
    pacmod3_msgs::msg::SteeringCmd steer_cmd;
    steer_cmd.header.frame_id = base_frame_id_;
    steer_cmd.header.stamp = current_time;
    steer_cmd.enable = engage_cmd_;
    steer_cmd.ignore_overrides = false;
    steer_cmd.clear_override = clear_override;
    steer_cmd.rotation_rate = calcSteerWheelRateCmd(adaptive_gear_ratio);
    steer_cmd.command = steerWheelRateLimiter(
      desired_steer_wheel, prev_steer_cmd_.command, current_time, prev_steer_cmd_.header.stamp,
      steer_cmd.rotation_rate, current_steer_wheel, engage_cmd_);
    steer_cmd_pub_->publish(steer_cmd);
    prev_steer_cmd_ = steer_cmd;
  }

  /* publish raw steering cmd for debug */
  {
    pacmod3_msgs::msg::SteeringCmd raw_steer_cmd;
    raw_steer_cmd.header.frame_id = base_frame_id_;
    raw_steer_cmd.header.stamp = current_time;
    raw_steer_cmd.enable = engage_cmd_;
    raw_steer_cmd.ignore_overrides = false;
    raw_steer_cmd.clear_override = clear_override;
    raw_steer_cmd.command = desired_steer_wheel;
    raw_steer_cmd.rotation_rate =
      control_cmd_ptr_->lateral.steering_tire_rotation_rate * adaptive_gear_ratio;
    raw_steer_cmd_pub_->publish(raw_steer_cmd);
  }

  /* publish shift cmd */
  {
    pacmod3_msgs::msg::SystemCmdInt shift_cmd;
    shift_cmd.header.frame_id = base_frame_id_;
    shift_cmd.header.stamp = current_time;
    shift_cmd.enable = engage_cmd_;
    shift_cmd.ignore_overrides = false;
    shift_cmd.clear_override = clear_override;
    shift_cmd.command = desired_shift;
    shift_cmd_pub_->publish(shift_cmd);
  }
}

double ExternalWatchdog::calculateVehicleVelocity(
  const pacmod3_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
  const pacmod3_msgs::msg::SystemRptInt & shift_rpt)
{
  const double sign = (shift_rpt.output == pacmod3_msgs::msg::SystemRptInt::SHIFT_REVERSE) ? -1 : 1;
  const double vel =
    (wheel_speed_rpt.rear_left_wheel_speed + wheel_speed_rpt.rear_right_wheel_speed) * 0.5 *
    tire_radius_ * speed_scale_factor_;
  return sign * vel;
}

double ExternalWatchdog::calculateVariableGearRatio(const double vel, const double steer_wheel)
{
  return std::max(
    1e-5, vgr_coef_a_ + vgr_coef_b_ * vel * vel - vgr_coef_c_ * std::fabs(steer_wheel));
}

double ExternalWatchdog::calcSteerWheelRateCmd(const double gear_ratio)
{
  const auto current_vel =
    std::fabs(calculateVehicleVelocity(*wheel_speed_rpt_ptr_, *gear_cmd_rpt_ptr_));

  // send low steer rate at low speed
  if (current_vel < std::numeric_limits<double>::epsilon()) {
    return steering_wheel_rate_stopped_;
  } else if (current_vel < low_vel_thresh_) {
    return steering_wheel_rate_low_vel_;
  }

  if (!enable_steering_rate_control_) {
    return max_steering_wheel_rate_;
  }

  constexpr double margin = 1.5;
  const double rate = margin * control_cmd_ptr_->lateral.steering_tire_rotation_rate * gear_ratio;
  return std::min(std::max(std::fabs(rate), min_steering_wheel_rate_), max_steering_wheel_rate_);
}

double ExternalWatchdog::steerWheelRateLimiter(
  const double current_steer_cmd, const double prev_steer_cmd,
  const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
  const double steer_rate, const double current_steer_output, const bool engage)
{
  if (!engage) {
    // return current steer as steer command ( do not apply steer rate filter )
    return current_steer_output;
  }

  const double dsteer = current_steer_cmd - prev_steer_cmd;
  const double dt = std::max(0.0, (current_steer_time - prev_steer_time).seconds());
  const double max_dsteer = std::fabs(steer_rate) * dt;
  const double limited_steer_cmd =
    prev_steer_cmd + std::min(std::max(-max_dsteer, dsteer), max_dsteer);
  return limited_steer_cmd;
}
