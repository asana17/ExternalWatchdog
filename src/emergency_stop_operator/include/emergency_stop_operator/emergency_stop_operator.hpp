#ifndef EMERGENCY_STOP_OPERATOR__EMERGENCY_STOP_OPERATOR_CORE_HPP_
#define EMERGENCY_STOP_OPERATOR__EMERGENCY_STOP_OPERATOR_CORE_HPP_

// Core
#include <functional>
#include <memory>

// Autoware
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <autoware_auto_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS2 core
#include <rclcpp/rclcpp.hpp>

namespace emergency_stop_operator
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_auto_vehicle_msgs::msg::GearCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_system_msgs::srv::OperateMrm;

struct Parameters
{
  int update_rate;             // [Hz]
  double target_acceleration;  // [m/s^2]
  double target_jerk;          // [m/s^3]
  bool use_parking_after_stopped;
};

class EmergencyStopOperator : public rclcpp::Node
{
public:
  explicit EmergencyStopOperator(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;

  // Subscriber
  // previous control cmd on Autoware ECU
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_report_;

  void onControlCommand(AckermannControlCommand::ConstSharedPtr msg);
  void onVelocityReport(VelocityReport::ConstSharedPtr msg);

  VelocityReport::ConstSharedPtr velocity_report_;

  // Server
  rclcpp::Service<OperateMrm>::SharedPtr service_operation_;

  void operateEmergencyStop(
    const OperateMrm::Request::SharedPtr request, const OperateMrm::Response::SharedPtr response);

  // Publisher
  rclcpp::Publisher<MrmBehaviorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_cmd_;
  rclcpp::Publisher<GearCommand>::SharedPtr pub_gear_cmd_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr pub_hazard_lights_cmd_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr pub_turn_indicators_cmd_;

  void publishStatus() const;
  void publishVehicleCommands() const;
  void publishControlCommand(const AckermannControlCommand & command) const;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();

  // States
  MrmBehaviorStatus status_;
  AckermannControlCommand prev_control_cmd_;
  bool is_prev_control_cmd_subscribed_;

  // Algorithm
  AckermannControlCommand calcTargetAcceleration(
    const AckermannControlCommand & prev_control_cmd) const;


  bool isStopped() const;
};

}  // namespace emergency_stop_operator

#endif  // EMERGENCY_STOP_OPERATOR__EMERGENCY_STOP_OPERATOR_CORE_HPP_
