#ifndef TILDE_ERROR_MONITOR__TILDE_ERROR_MONITOR_CORE_HPP_
#define TILDE_ERROR_MONITOR__TILDE_ERROR_MONITOR_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <watchdog_system_msgs/msg/tilde_diagnostic_array.hpp>
#include <watchdog_system_msgs/msg/tilde_hazard_status_stamped.hpp>

#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct TildeDiagStamped
{
  std_msgs::msg::Header header;  // used for timeout check
  watchdog_system_msgs::msg::TildeDiagnosticStatus status;
};

using TildeDiagBuffer = std::deque<TildeDiagStamped>;

struct TildeDiagConfig
{
  std::string start_point;
  std::string end_point;
  std::string sf_at;
  std::string lf_at;
  std::string spf_at;
};

using RequiredPaths = std::vector<TildeDiagConfig>;

struct KeyName
{
  static constexpr const char * autonomous_driving = "autonomous_driving";
  static constexpr const char * test_sensing = "test_sensing";
};

class TildeErrorMonitor : public rclcpp::Node
{
public:
  TildeErrorMonitor();

private:
  struct Parameters
  {
    int update_rate;
    bool ignore_missing_tilde_diagnostics;
    double data_ready_timeout;
    double tilde_diag_timeout_sec;
    int emergency_tilde_hazard_level;
  };

  Parameters params_{};

  rclcpp::Time initialized_time_;
  watchdog_system_msgs::msg::TildeHazardStatus tilde_hazard_status_{};
  std::unordered_map<std::string, RequiredPaths> required_paths_map_;
  std::string current_mode_;

  void loadRequiredPaths(const std::string & key);

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  void onTimer();

  // Subscriber
  rclcpp::Subscription<watchdog_system_msgs::msg::TildeDiagnosticArray>::SharedPtr
    sub_tilde_diag_array_;

  void onDiagArray(
    const watchdog_system_msgs::msg::TildeDiagnosticArray::ConstSharedPtr msg);  // msg parser

  const size_t tilde_diag_buffer_size_ = 50;
  std::unordered_map<std::string, TildeDiagBuffer> tilde_diag_buffer_map_;
  watchdog_system_msgs::msg::TildeDiagnosticArray::ConstSharedPtr tilde_diag_array_;

  // Publisher
  rclcpp::Publisher<watchdog_system_msgs::msg::TildeHazardStatusStamped>::SharedPtr
    pub_tilde_hazard_status_;
  void publishTildeHazardStatus(
    const watchdog_system_msgs::msg::TildeHazardStatus & tilde_hazard_status);

  // get hazard status from diagnostics
  std::optional<TildeDiagStamped> getLatestTildeDiag(const std::string & tilde_path_name) const;
  uint8_t getTildeHazardLevel(
    const TildeDiagConfig & required_path, const int tild_diag_level) const;
  void appendTildeHazardDiag(
    const TildeDiagConfig & required_path,
    const watchdog_system_msgs::msg::TildeDiagnosticStatus & tilde_hazard_diag,
    watchdog_system_msgs::msg::TildeHazardStatus * tilde_hazard_status) const;
  watchdog_system_msgs::msg::TildeHazardStatus judgeTildeHazardStatus() const;
  void updateTildeHazardStatus();
};

#endif  // TILDE_ERROR_MONITOR__TILDE_ERROR_MONITOR_CORE_HPP_
