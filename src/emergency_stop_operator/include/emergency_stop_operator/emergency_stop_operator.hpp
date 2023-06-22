#ifndef EMERGENCY_STOP_OPERATOR__EMERGENCY_STOP_OPERATOR_CORE_HPP_
#define EMERGENCY_STOP_OPERATOR__EMERGENCY_STOP_OPERATOR_CORE_HPP_

// Core
#include <functional>
#include <memory>

// Autoware
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>

// ROS2 core
#include <rclcpp/rclcpp.hpp>

namespace emergency_stop_operator
{
using autoware_auto_control_msgs::msg::AckermannControlCommand;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_system_msgs::srv::OperateMrm;

struct Parameters
{
  int update_rate;             // [Hz]
  double target_acceleration;  // [m/s^2]
  double target_jerk;          // [m/s^3]
};

class EmergencyStopOperator : public rclcpp::Node
{
public:
  explicit EmergencyStopOperator(const rclcpp::NodeOptions & node_options);

private:
  // Parameters
  Parameters params_;

  // Subscriber
  rclcpp::Subscription<AckermannControlCommand>::SharedPtr sub_control_cmd_;

  void onControlCommand(AckermannControlCommand::ConstSharedPtr msg);

  // Server
  rclcpp::Service<OperateMrm>::SharedPtr service_operation_;

  void operateEmergencyStop(
    const OperateMrm::Request::SharedPtr request, const OperateMrm::Response::SharedPtr response);

  // Publisher
  rclcpp::Publisher<MrmBehaviorStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<AckermannControlCommand>::SharedPtr pub_control_cmd_;

  void publishStatus() const;
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
};

}  // namespace emergency_stop_operator

#endif  // EMERGENCY_STOP_OPERATOR__EMERGENCY_STOP_OPERATOR_CORE_HPP_
