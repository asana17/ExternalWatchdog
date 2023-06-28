#ifndef DIAGNOSTIC_ERROR_MONITOR__DIAGNOSTIC_ERROR_MONITOR_CORE_HPP_
#define DIAGNOSTIC_ERROR_MONITOR__DIAGNOSTIC_ERROR_MONITOR_CORE_HPP_

#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <watchdog_system_msgs/msg/hazard_status_stamped.hpp>

#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct DiagStamped
{
  std_msgs::msg::Header header;
  diagnostic_msgs::msg::DiagnosticStatus status;
};

using DiagBuffer = std::deque<DiagStamped>;

struct DiagConfig
{
  std::string name;
  std::string sf_at;
  std::string lf_at;
  std::string spf_at;
  bool auto_recovery;
  bool self_recoverable;
};

using RequiredModules = std::vector<DiagConfig>;

struct KeyName
{
  static constexpr const char * autonomous_driving = "autonomous_driving";
};

class DiagnosticErrorMonitor : public rclcpp::Node
{
public:
  DiagnosticErrorMonitor();

private:
  struct Parameters
  {
    int update_rate;
    bool ignore_missing_diagnostics;
    bool add_leaf_diagnostics;
    double data_ready_timeout;
    double diag_timeout_sec;
    double hazard_recovery_timeout;
    int emergency_hazard_level;
    bool use_emergency_hold;
  };

  Parameters params_{};

  rclcpp::Time emergency_state_switch_time_;
  rclcpp::Time initialized_time_;
  watchdog_system_msgs::msg::HazardStatus hazard_status_{};
  std::unordered_map<std::string, RequiredModules> required_modules_map_;
  std::string current_mode_;

  void loadRequiredModules(const std::string & key);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  void onTimer();

  // subscriber
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_diag_array_;

  void onDiagArray(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg);  // msg parser

  const size_t diag_buffer_size_ = 100;
  std::unordered_map<std::string, DiagBuffer> diag_buffer_map_;  // map of (header, diag status)
  diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_array_;  // ptr for msg

  // publisher
  rclcpp::Publisher<watchdog_system_msgs::msg::HazardStatusStamped>::SharedPtr pub_hazard_status_;
  void publishHazardStatus(const watchdog_system_msgs::msg::HazardStatus & hazard_status);

  // get hazard status from diagnostics

  std::optional<DiagStamped> getLatestDiag(const std::string & diag_name) const;
  uint8_t getHazardLevel(const DiagConfig & required_module, const int diag_level) const;
  bool isSelfRecoverable(const DiagConfig & required_module) const;
  void appendHazardDiag(
    const DiagConfig & required_module, const diagnostic_msgs::msg::DiagnosticStatus & diag,
    watchdog_system_msgs::msg::HazardStatus * hazard_status) const;
  watchdog_system_msgs::msg::HazardStatus judgeHazardStatus() const;
  void updateHazardStatus();
  bool canAutoRecovery() const;
  bool isEmergencyHoldingRequired() const;
};

#endif  // DIAGNOSTIC_ERROR_MONITOR__DIAGNOSTIC_ERROR_MONITOR_CORE_HPP_
