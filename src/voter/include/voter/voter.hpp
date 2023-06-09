#ifndef VOTER__VOTER_HPP_
#define VOTER__VOTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <tier4_system_msgs/msg/mrm_behavior_status.hpp>
#include <tier4_system_msgs/srv/operate_mrm.hpp>
#include "watchdog_system_msgs/msg/hazard_status_stamped.hpp"
#include <functional>


namespace voter{

using watchdog_system_msgs::msg::HazardStatusStamped;
using tier4_system_msgs::msg::MrmBehaviorStatus;
using tier4_system_msgs::srv::OperateMrm;

enum ecu_name {
  Main, Sub, Supervisor
};

struct ECU {
  ecu_name name;

  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_self_monitoring_;
  rclcpp::Subscription<HazardStatusStamped>::SharedPtr sub_external_monitoring_;
  rclcpp::Subscription<MrmBehaviorStatus>::SharedPtr sub_mrm_comfortable_stop_status_;
  rclcpp::Subscription<MrmBehaviorStatus>::SharedPtr sub_mrm_sudden_stop_status_;
  HazardStatusStamped::ConstSharedPtr self_hazard_status_stamped_;
  HazardStatusStamped::ConstSharedPtr external_hazard_status_stamped_;

  MrmBehaviorStatus::ConstSharedPtr mrm_comfortable_stop_status_;
  MrmBehaviorStatus::ConstSharedPtr mrm_sudden_stop_status_;

  rclcpp::CallbackGroup::SharedPtr client_mrm_comfortable_stop_group_;
  rclcpp::Client<OperateMrm>::SharedPtr client_mrm_comfortable_stop_;
  rclcpp::CallbackGroup::SharedPtr client_mrm_sudden_stop_group_;
  rclcpp::Client<OperateMrm>::SharedPtr client_mrm_sudden_stop_;

};

struct Param
{
  int update_rate;
  double timeout_hazard_status;
};
class Voter : public rclcpp::Node
{

public:
  Voter();

private:

  ECU Main_, Sub_, Supervisor_;

  static void onSelfMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, ECU* ecu);
  static void onExternalMonitoringStamped(
    const HazardStatusStamped::ConstSharedPtr msg, ECU* ecu);
  static void onMrmComfortableStopStatus(
    const MrmBehaviorStatus::ConstSharedPtr msg, ECU* ecu);
  static void onMrmSuddenStopStatus(
    const MrmBehaviorStatus::ConstSharedPtr msg, ECU* ecu);


  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void onTimer();

  bool isDataReady();
  bool isEcuDataReady(const ECU* ecu);

  //Paramters
  Param param_;

  void callMrmBehavior(
      const autoware_adapi_v1_msgs::msg::MrmState::_behavior_type & mrm_behavior) const;


};

}

#endif
