#ifndef EXTERNAL_WATCHDOG__EXTERNAL_WATCHDOG_CORE_HPP_
#define EXTERNAL_WATCHDOG__EXTERNAL_WATCHDOG_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "vehicle_info_util/vehicle_info_util.hpp"

#include "watchdog_system_msgs/msg/hazard_status_stamped.hpp"
#include "watchdog_system_msgs/msg/tilde_hazard_status_stamped.hpp"
#include "watchdog_system_msgs/msg/ackermann_control_command.hpp"


#include <nav_msgs/msg/odometry.hpp>

#include <pacmod3_msgs/msg/global_rpt.hpp>
#include <pacmod3_msgs/msg/steering_cmd.hpp>
#include <pacmod3_msgs/msg/system_cmd_float.hpp>
#include <pacmod3_msgs/msg/system_cmd_int.hpp>
#include <pacmod3_msgs/msg/system_rpt_float.hpp>
#include <pacmod3_msgs/msg/system_rpt_int.hpp>
#include <pacmod3_msgs/msg/wheel_speed_rpt.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <string>
#include <optional>



class ExternalWatchdog : public rclcpp::Node
{
public:
  ExternalWatchdog();

private:

  typedef message_filters::sync_policies::ApproximateTime<
    pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::WheelSpeedRpt,
    pacmod3_msgs::msg::SystemRptFloat, pacmod3_msgs::msg::SystemRptFloat,
    pacmod3_msgs::msg::SystemRptInt, pacmod3_msgs::msg::SystemRptInt, pacmod3_msgs::msg::GlobalRpt,
    pacmod3_msgs::msg::SystemRptInt>
    PacmodFeedbacksSyncPolicy;

  struct Parameters
  {
    int update_rate;
    double data_timeout;
  };

  Parameters params_{};

  watchdog_system_msgs::msg::HazardStatusStamped::ConstSharedPtr hazard_status_;
  watchdog_system_msgs::msg::TildeHazardStatusStamped::ConstSharedPtr tilde_hazard_status_;
  watchdog_system_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
  nav_msgs::msg::Odometry::ConstSharedPtr odom_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  // Subscriber
  rclcpp::Subscription<watchdog_system_msgs::msg::HazardStatusStamped>::SharedPtr
    sub_hazard_status_;
  rclcpp::Subscription<watchdog_system_msgs::msg::TildeHazardStatusStamped>::SharedPtr
    sub_tilde_hazard_status_;
  rclcpp::Subscription<watchdog_system_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_prev_control_command_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

  rclcpp::Time hazard_status_sub_time_;
  rclcpp::Time tilde_hazard_status_sub_time_;

  // From Pacmod
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>>
    steer_wheel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::WheelSpeedRpt>>
    wheel_speed_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> accel_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptFloat>> brake_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> shift_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> turn_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::GlobalRpt>> global_rpt_sub_;
  std::unique_ptr<message_filters::Subscriber<pacmod3_msgs::msg::SystemRptInt>> rear_door_rpt_sub_;
  std::unique_ptr<message_filters::Synchronizer<PacmodFeedbacksSyncPolicy>> pacmod_feedbacks_sync_;



/* publishers */
  // To Pacmod
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr accel_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdFloat>::SharedPtr brake_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr steer_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr shift_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr turn_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SystemCmdInt>::SharedPtr door_cmd_pub_;
  rclcpp::Publisher<pacmod3_msgs::msg::SteeringCmd>::SharedPtr
    raw_steer_cmd_pub_;  // only for debug
  

  /* ros param */
  std::string base_frame_id_;

  bool is_pacmod_rpt_received_ = false;
  bool is_pacmod_enabled_ = false;
  bool is_clear_override_needed_ = false;
  double tire_radius_;         // [m]
  double wheel_base_;          // [m]
  double steering_offset_;     // [rad] def: measured = truth + offset
  double vgr_coef_a_;          // variable gear ratio coeffs
  double vgr_coef_b_;          // variable gear ratio coeffs
  double vgr_coef_c_;          // variable gear ratio coeffs
  double accel_pedal_offset_;  // offset of accel pedal value
  double brake_pedal_offset_;  // offset of brake pedal value
  double speed_scale_factor_;  // scale factor of speed

  double emergency_brake_;              // brake command when emergency [m/s^2]
  double max_throttle_;                 // max throttle [0~1]
  double max_brake_;                    // max throttle [0~1]
  double max_steering_wheel_;           // max steering wheel angle [rad]
  double max_steering_wheel_rate_;      // [rad/s]
  double min_steering_wheel_rate_;      // [rad/s]
  double steering_wheel_rate_low_vel_;  // [rad/s]
  double steering_wheel_rate_stopped_;  // [rad/s]
  double low_vel_thresh_;               // [m/s]
                                        //
  bool enable_steering_rate_control_;  // use steering angle speed for command [rad/s]
                                        //
  vehicle_info_util::VehicleInfo vehicle_info_;

  /* input values */

  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt_ptr_;  // [rad]
  pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt_ptr_;   // [m/s]
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt_ptr_;
  pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt_ptr_;   // [m/s]
  pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr gear_cmd_rpt_ptr_;  // [m/s]
  pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt_ptr_;       // [m/s]
  pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt_ptr_;
  pacmod3_msgs::msg::SteeringCmd prev_steer_cmd_;

  bool engage_cmd_{false};


  /* functions */
  void onHazardStatus(const watchdog_system_msgs::msg::HazardStatusStamped::ConstSharedPtr msg);
  void onTildeHazardStatus(const watchdog_system_msgs::msg::TildeHazardStatusStamped::ConstSharedPtr msg);
  void onPrevControlCommand(
    const watchdog_system_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void judgeHazardStatus(const uint8_t hazard_status_level);

  void callDirectMRM();

  bool isStopped();

  void callbackPacmodRpt(
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr steer_wheel_rpt,
    const pacmod3_msgs::msg::WheelSpeedRpt::ConstSharedPtr wheel_speed_rpt,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr accel_rpt,
    const pacmod3_msgs::msg::SystemRptFloat::ConstSharedPtr brake_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr gear_cmd_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr turn_rpt,
    const pacmod3_msgs::msg::GlobalRpt::ConstSharedPtr global_rpt,
    const pacmod3_msgs::msg::SystemRptInt::ConstSharedPtr rear_door_rpt);

  void publishPacmodCommands();
  double calculateVehicleVelocity(
    const pacmod3_msgs::msg::WheelSpeedRpt & wheel_speed_rpt,
    const pacmod3_msgs::msg::SystemRptInt & shift_rpt);
  double calculateVariableGearRatio(const double vel, const double steer_wheel);
  double calcSteerWheelRateCmd(const double gear_ratio);

  double steerWheelRateLimiter(
    const double current_steer_cmd, const double prev_steer_cmd,
    const rclcpp::Time & current_steer_time, const rclcpp::Time & prev_steer_time,
    const double steer_rate, const double current_steer_output, const bool engage);

  //watchdog_system_msgs::msg::AckermannControlCommand getEmergencyControlCommand();
};

#endif  // EXTERNAL_WATCHDOG__EXTERNAL_WATCHDOG_CORE_HPP_
