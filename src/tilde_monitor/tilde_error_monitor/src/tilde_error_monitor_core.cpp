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

#include "tilde_error_monitor/tilde_error_monitor_core.hpp"

#include <regex>
#include <set>
#include <string>
#include <vector>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

namespace
{

std::vector<std::string> split(const std::string & str, const char delim)
{
  std::vector<std::string> elems;
  std::stringstream ss(str);
  std::string item;
  while (std::getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

std::string getPathName(const std::string & start_point, const std::string & end_point)
{
  std::string path = start_point + "-" + end_point;
  return path;
}

int str2level(const std::string & level_str)
{
  using std::regex_constants::icase;
  using watchdog_system_msgs::msg::TildeDiagnosticStatus;

  if (std::regex_match(level_str, std::regex("warn", icase))) {
    return TildeDiagnosticStatus::WARN;
  }
  if (std::regex_match(level_str, std::regex("error", icase))) {
    return TildeDiagnosticStatus::ERROR;
  }
  if (std::regex_match(level_str, std::regex("stale", icase))) {
    return TildeDiagnosticStatus::STALE;
  }

  throw std::runtime_error(fmt::format("invalid level: {}", level_str));
}

bool isOverLevel(const int & tilde_diag_level, const std::string & failure_level_str)
{
  if (failure_level_str == "none") {
    return false;
  }

  return tilde_diag_level >= str2level(failure_level_str);
}

// Get ref of target hazard level tilde diagnostic array in message
std::vector<watchdog_system_msgs::msg::TildeDiagnosticStatus> & getTargetTildeDiagnosticsRef(
  const int tilde_hazard_level, watchdog_system_msgs::msg::TildeHazardStatus * tilde_hazard_status)
{
  using watchdog_system_msgs::msg::TildeHazardStatus;

  if (tilde_hazard_level == TildeHazardStatus::NO_FAULT) {
    return tilde_hazard_status->tilde_diag_no_fault;
  }
  if (tilde_hazard_level == TildeHazardStatus::SAFE_FAULT) {
    return tilde_hazard_status->tilde_diag_safe_fault;
  }
  if (tilde_hazard_level == TildeHazardStatus::LATENT_FAULT) {
    return tilde_hazard_status->tilde_diag_latent_fault;
  }
  if (tilde_hazard_level == TildeHazardStatus::SINGLE_POINT_FAULT) {
    return tilde_hazard_status->tilde_diag_single_point_fault;
  }

  throw std::runtime_error(fmt::format("invalid tilde hazard level: {}", tilde_hazard_level));
}

watchdog_system_msgs::msg::TildeHazardStatus createTimeoutTildeHazardStatus()
{
  watchdog_system_msgs::msg::TildeHazardStatus tilde_hazard_status;
  tilde_hazard_status.level = watchdog_system_msgs::msg::TildeHazardStatus::SINGLE_POINT_FAULT;
  tilde_hazard_status.emergency = true;
  watchdog_system_msgs::msg::TildeDiagnosticStatus tilde_diag;
  tilde_diag.start_point = "tilde_error_monitor/input_data_timeout";
  tilde_diag.end_point = "";
  tilde_diag.level = watchdog_system_msgs::msg::TildeDiagnosticStatus::ERROR;
  tilde_hazard_status.tilde_diag_single_point_fault.push_back(tilde_diag);
  return tilde_hazard_status;
}

}  // namespace

TildeErrorMonitor::TildeErrorMonitor()
: Node(
    "tilde_error_monitor",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  // Parameter
  get_parameter_or<int>("update_rate", params_.update_rate, 10);
  get_parameter_or<bool>(
    "ignore_missing_tilde_diagnostics", params_.ignore_missing_tilde_diagnostics, false);
  get_parameter_or<double>("tilde_diag_timeout_sec", params_.tilde_diag_timeout_sec, 1.0);
  get_parameter_or<double>("data_ready_timeout", params_.data_ready_timeout, 30.0);
  get_parameter_or<int>(
    "emergency_tilde_hazard_level", params_.emergency_tilde_hazard_level,
    watchdog_system_msgs::msg::TildeHazardStatus::LATENT_FAULT);

  loadRequiredPaths(KeyName::test_sensing);

  using std::placeholders::_1;

  // Subscriber
  sub_tilde_diag_array_ = create_subscription<watchdog_system_msgs::msg::TildeDiagnosticArray>(
    "input/tilde_diag_array", rclcpp::QoS{1}, std::bind(&TildeErrorMonitor::onDiagArray, this, _1));

  //Publisher
  pub_tilde_hazard_status_ = create_publisher<watchdog_system_msgs::msg::TildeHazardStatusStamped>(
    "~/output/tilde_hazard_status", rclcpp::QoS{1});

  initialized_time_ = this->now();
  const auto period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&TildeErrorMonitor::onTimer, this));
}

// load required paths and params from config yaml
void TildeErrorMonitor::loadRequiredPaths(const std::string & key)
{
  const auto param_key = std::string("required_paths.") + key;

  const uint64_t depth = 4;
  const auto param_names = this->list_parameters({param_key}, depth).names;

  if (param_names.empty()) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  // Load path names from parameter key
  std::set<std::string> path_names;
  RequiredPaths required_paths;

  for (const auto & param_name : param_names) {
    // Example of param_name: required_paths.key.end_point.start_point
    //                    or  required_paths.key.end_point.start_point.parameter
    const auto split_names = split(param_name, '.');
    const auto & param_required_paths = split_names.at(0);
    const auto & param_key = split_names.at(1);
    const auto & param_end_point = split_names.at(2);
    const auto & param_start_point = split_names.at(3);

    const auto & path_name_with_prefix = fmt::format(
      "{0}.{1}.{2}.{3}", param_required_paths, param_key, param_end_point, param_start_point);

    if (path_names.count(path_name_with_prefix) != 0) {
      continue;  // Skip duprecated path
    }

    // Register name
    path_names.insert(path_name_with_prefix);

    // Load diag level
    const auto sf_key = path_name_with_prefix + std::string(".sf_at");
    std::string sf_at;
    this->get_parameter_or(sf_key, sf_at, std::string("none"));
    const auto lf_key = path_name_with_prefix + std::string(".lf_at");
    std::string lf_at;
    this->get_parameter_or(lf_key, lf_at, std::string("warn"));
    const auto spf_key = path_name_with_prefix + std::string(".spf_at");
    std::string spf_at;
    this->get_parameter_or(spf_key, spf_at, std::string("error"));

    // Register each path
    required_paths.push_back({param_start_point, param_end_point, sf_at, lf_at, spf_at});
  }

  required_paths_map_.insert(std::make_pair(key, required_paths));
}

// Register diag and msg from message to map
void TildeErrorMonitor::onDiagArray(
  const watchdog_system_msgs::msg::TildeDiagnosticArray::ConstSharedPtr msg)
{
  tilde_diag_array_ = msg;

  const auto & header = msg->header;

  for (const auto & tilde_diag : msg->status) {
    const std::string tilde_path_name = getPathName(tilde_diag.start_point, tilde_diag.end_point);
    if (tilde_diag_buffer_map_.count(tilde_path_name) == 0) {
      tilde_diag_buffer_map_.insert(std::make_pair(tilde_path_name, TildeDiagBuffer{}));
    }

    auto & tilde_diag_buffer = tilde_diag_buffer_map_.at(tilde_path_name);
    tilde_diag_buffer.push_back(TildeDiagStamped{header, tilde_diag});

    while (tilde_diag_buffer.size() > tilde_diag_buffer_size_) {
      tilde_diag_buffer.pop_front();
    }
  }
}

// Timer functions
bool TildeErrorMonitor::isDataReady()
{
  if (!tilde_diag_array_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for tilde_diag_array msg...");
    return false;
  }
  return true;
}

void TildeErrorMonitor::onTimer()
{
  if (!isDataReady()) {
    if ((this->now() - initialized_time_).seconds() > params_.data_ready_timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(),
        "input data is timeout");
      publishTildeHazardStatus(createTimeoutTildeHazardStatus());
    }
    return;
  }
  current_mode_ = KeyName::test_sensing;
  updateTildeHazardStatus();
  publishTildeHazardStatus(tilde_hazard_status_);
}

// get latest diag from map
std::optional<TildeDiagStamped> TildeErrorMonitor::getLatestTildeDiag(
  const std::string & tilde_path_name) const
{
  if (tilde_diag_buffer_map_.count(tilde_path_name) == 0) {
    return {};
  }

  const auto & tilde_diag_buffer = tilde_diag_buffer_map_.at(tilde_path_name);

  if (tilde_diag_buffer.empty()) {
    return {};
  }

  return tilde_diag_buffer.back();
}

uint8_t TildeErrorMonitor::getTildeHazardLevel(
  const TildeDiagConfig & required_path, const int tilde_diag_level) const
{
  using watchdog_system_msgs::msg::TildeHazardStatus;

  if (isOverLevel(tilde_diag_level, required_path.spf_at)) {
    return TildeHazardStatus::SINGLE_POINT_FAULT;
  }
  if (isOverLevel(tilde_diag_level, required_path.lf_at)) {
    return TildeHazardStatus::LATENT_FAULT;
  }
  if (isOverLevel(tilde_diag_level, required_path.sf_at)) {
    return TildeHazardStatus::SAFE_FAULT;
  }

  return TildeHazardStatus::NO_FAULT;
}

// Register TildeDiagnosticStatus to TildeHazardStatus by each hazard level
void TildeErrorMonitor::appendTildeHazardDiag(
  const TildeDiagConfig & required_path,
  const watchdog_system_msgs::msg::TildeDiagnosticStatus & tilde_hazard_diag,
  watchdog_system_msgs::msg::TildeHazardStatus * tilde_hazard_status) const
{
  const auto tilde_hazard_level = getTildeHazardLevel(required_path, tilde_hazard_diag.level);

  // Get target tilde diagnostic array ref
  auto & target_tilde_diagnostics_ref =
    getTargetTildeDiagnosticsRef(tilde_hazard_level, tilde_hazard_status);
  // Regiter to ref
  target_tilde_diagnostics_ref.push_back(tilde_hazard_diag);

  tilde_hazard_status->level = std::max(tilde_hazard_status->level, tilde_hazard_level);
}

watchdog_system_msgs::msg::TildeHazardStatus TildeErrorMonitor::judgeTildeHazardStatus() const
{
  using watchdog_system_msgs::msg::TildeDiagnosticStatus;
  using watchdog_system_msgs::msg::TildeHazardStatus;


  TildeHazardStatus tilde_hazard_status;

  for (const auto & required_path : required_paths_map_.at(current_mode_)) {
    const auto & diag_start_point = required_path.start_point;
    const auto & diag_end_point = required_path.end_point;
    const std::string diag_path_name = getPathName(diag_start_point, diag_end_point);
    const auto latest_tilde_diag = getLatestTildeDiag(diag_path_name);

    // no diag found
    if (!latest_tilde_diag) {
      if (!params_.ignore_missing_tilde_diagnostics) {
        TildeDiagnosticStatus missing_tilde_diag;
        missing_tilde_diag.start_point = diag_start_point;
        missing_tilde_diag.end_point = diag_end_point;
        missing_tilde_diag.level = TildeDiagnosticStatus::STALE;
        missing_tilde_diag.message = "no tilde diag found";

        appendTildeHazardDiag(required_path, missing_tilde_diag, &tilde_hazard_status);
      }

      continue;
    }

    // diag level high
    {
      appendTildeHazardDiag(required_path, latest_tilde_diag->status, &tilde_hazard_status);
    }

    // diag timeout
    {
      const auto time_diff = this->now() - latest_tilde_diag->header.stamp;
      if (time_diff.seconds() > params_.tilde_diag_timeout_sec) {
        TildeDiagnosticStatus timeout_tilde_diag = latest_tilde_diag->status;
        timeout_tilde_diag.level = TildeDiagnosticStatus::STALE;
        timeout_tilde_diag.message = "timeout";

        appendTildeHazardDiag(required_path, timeout_tilde_diag, &tilde_hazard_status);
      }
    }
  }

  return tilde_hazard_status;
}

void TildeErrorMonitor::updateTildeHazardStatus()
{
  // const bool prev_emergency_status = hazard_status_.emergency;

  // Create hazard status based on diagnostics
  const auto current_tilde_hazard_status = judgeTildeHazardStatus();
  tilde_hazard_status_.level = current_tilde_hazard_status.level;
  tilde_hazard_status_.tilde_diag_no_fault = current_tilde_hazard_status.tilde_diag_no_fault;
  tilde_hazard_status_.tilde_diag_safe_fault = current_tilde_hazard_status.tilde_diag_safe_fault;
  tilde_hazard_status_.tilde_diag_latent_fault =
    current_tilde_hazard_status.tilde_diag_latent_fault;
  tilde_hazard_status_.tilde_diag_single_point_fault =
    current_tilde_hazard_status.tilde_diag_single_point_fault;

  // Update emergency status
  {
    tilde_hazard_status_.emergency =
      tilde_hazard_status_.level >= params_.emergency_tilde_hazard_level;
  }

}

void TildeErrorMonitor::publishTildeHazardStatus(
  const watchdog_system_msgs::msg::TildeHazardStatus & tilde_hazard_status)
{
  watchdog_system_msgs::msg::TildeHazardStatusStamped tilde_hazard_status_stamped;
  tilde_hazard_status_stamped.stamp = this->now();
  tilde_hazard_status_stamped.status = tilde_hazard_status;
  pub_tilde_hazard_status_->publish(tilde_hazard_status_stamped);
}
