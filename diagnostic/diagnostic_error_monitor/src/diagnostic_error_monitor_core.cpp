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

#include <algorithm>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <vector>

#define FMT_HEADER_ONLY
#include "diagnostic_error_monitor/diagnostics_filter.hpp"
#include "diagnostic_error_monitor/diagnostic_error_monitor_core.hpp"

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

int str2level(const std::string & level_str)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using std::regex_constants::icase;

  if (std::regex_match(level_str, std::regex("warn", icase))) {
    return DiagnosticStatus::WARN;
  }
  if (std::regex_match(level_str, std::regex("error", icase))) {
    return DiagnosticStatus::ERROR;
  }
  if (std::regex_match(level_str, std::regex("stale", icase))) {
    return DiagnosticStatus::STALE;
  }

  throw std::runtime_error(fmt::format("invalid level: {}", level_str));
}

bool isOverLevel(const int & diag_level, const std::string & failure_level_str)
{
  if (failure_level_str == "none") {
    return false;
  }

  return diag_level >= str2level(failure_level_str);
}

// Get ref of target hazard level diagnostic array in message
std::vector<diagnostic_msgs::msg::DiagnosticStatus> & getTargetDiagnosticsRef(
    const int hazard_level, watchdog_system_msgs::msg::HazardStatus * hazard_status)
{
  using watchdog_system_msgs::msg::HazardStatus;

  if (hazard_level == HazardStatus::NO_FAULT) {
    return hazard_status->diag_no_fault;
  }
  if (hazard_level == HazardStatus::SAFE_FAULT) {
    return hazard_status->diag_safe_fault;
  }
  if (hazard_level == HazardStatus::LATENT_FAULT) {
    return hazard_status->diag_latent_fault;
  }
  if (hazard_level == HazardStatus::SINGLE_POINT_FAULT) {
    return hazard_status->diag_single_point_fault;
  }

  throw std::runtime_error(fmt::format("invalid hazard level: {}", hazard_level));
}

/*std::set<std::string> getErrorModules(
    const watchdog_system_msgs::msg::HazardStatus & hazard_status,
    const int emergency_hazard_level)
{
  std::set<std::string> error_modules;
  using watchdog_system_msgs::msg::HazardStatus;

  if (emergency_hazard_level <= HazardStatus::SINGLE_POINT_FAULT) {
    for (const auto & diag_spf : hazard_status.diag_single_point_fault) {
      error_modules.insert(diag_spf.name);
    }
  }
  if (emergency_hazard_level <= HazardStatus::LATENT_FAULT) {
    for (const auto & diag_lf : hazard_status.diag_latent_fault) {
      error_modules.insert(diag_lf.name);
    }
  }
  if (emergency_hazard_level <= HazardStatus::SAFE_FAULT) {
    for (const auto & diag_sf : hazard_status.diag_safe_fault) {
      error_modules.insert(diag_sf.name);
    }
  }

  return error_modules;
}*/

//Timeout error message
watchdog_system_msgs::msg::HazardStatus createTimeoutHazardStatus()
{
  watchdog_system_msgs::msg::HazardStatus hazard_status;
  hazard_status.level = watchdog_system_msgs::msg::HazardStatus::SINGLE_POINT_FAULT;
  hazard_status.emergency = true;
  hazard_status.emergency_holding = false;
  diagnostic_msgs::msg::DiagnosticStatus diag;
  diag.name = "diagnostic_error_monitor/input_data_timeout";
  diag.hardware_id = "diagnostic_error_monitor";
  diag.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  hazard_status.diag_single_point_fault.push_back(diag);
  return hazard_status;
}

}//namespace


DiagnosticErrorMonitor::DiagnosticErrorMonitor()
  : Node(
      "diagnostic_error_monitor",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  // Parameter
  get_parameter_or<int>("update_rate", params_.update_rate, 10);
  get_parameter_or<bool>("ignore_missing_diagnostics", params_.ignore_missing_diagnostics, false);
  get_parameter_or<bool>("add_leaf_diagnostics", params_.add_leaf_diagnostics, true);
  get_parameter_or<double>("data_ready_timeout", params_.data_ready_timeout, 30.0);
  get_parameter_or<double>("diag_timeout_sec", params_.diag_timeout_sec, 1.0);
  get_parameter_or<int>("emergency_hazard_level", params_.emergency_hazard_level, watchdog_system_msgs::msg::HazardStatus::LATENT_FAULT);
  get_parameter_or<bool>("use_emergency_hold", params_.use_emergency_hold, false);

  //TODO Define config yaml key name
  loadRequiredModules(KeyName::autonomous_driving);

  using std::placeholders::_1;

  // Subscriber
  sub_diag_array_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "input/diag_array", rclcpp::QoS{1}, std::bind(&DiagnosticErrorMonitor::onDiagArray, this, _1));

  // Publisher
  pub_hazard_status_ = create_publisher<watchdog_system_msgs::msg::HazardStatusStamped>(
      "~/output/diagnostic_hazard_status", rclcpp::QoS{1});

  // Timer
  initialized_time_ = this->now();
  const auto period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&DiagnosticErrorMonitor::onTimer, this));
}


// Load required modules and params from config yaml
void DiagnosticErrorMonitor::loadRequiredModules(const std::string & key)
{
  //TODO: Are two config keyname needed?
  const auto param_key = std::string("required_modules.") + key;

  const uint64_t depth = 3;
  const auto param_names = this->list_parameters({param_key}, depth).names;

  if (param_names.empty()) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }

  // Load module names from parameter key
  std::set<std::string> module_names;
  RequiredModules required_modules;

  for (const auto & param_name : param_names) {
    // Example of param_name: required_modules.key.module
    //                    or  required_modules.key.module.parameter
    const auto split_names = split(param_name, '.');
    const auto & param_required_modules = split_names.at(0);
    const auto & param_key = split_names.at(1);
    const auto & param_module = split_names.at(2);
    // TODO: param_required_modules is fixed to "required_modules"
    const auto module_name_with_prefix = 
      fmt::format("{0}.{1}.{2}", param_required_modules, param_key, param_module);

    if (module_names.count(module_name_with_prefix) != 0) {
      continue; // Skip duplicate parameter
    }

    //Register name
    module_names.insert(module_name_with_prefix);

    // Load diag level
    const auto sf_key = module_name_with_prefix + std::string(".sf_at");
    std::string sf_at;
    this->get_parameter_or(sf_key, sf_at, std::string("none"));
    const auto lf_key = module_name_with_prefix + std::string(".lf_at");
    std::string lf_at;
    this->get_parameter_or(lf_key, lf_at, std::string("warn"));
    const auto spf_key = module_name_with_prefix + std::string(".spf_at");
    std::string spf_at;
    this->get_parameter_or(spf_key, spf_at, std::string("error"));

    //auto_recovery
    const auto auto_recovery_key = module_name_with_prefix + std::string(".auto_recovery");
    std::string auto_recovery_approval_str;
    this->get_parameter_or(auto_recovery_key, auto_recovery_approval_str, std::string("true"));
    //convert str to bool
    bool auto_recovery_approval{};
    std::istringstream(auto_recovery_approval_str) >> std::boolalpha >> auto_recovery_approval;

    //Register each module
    required_modules.push_back({param_module, sf_at, lf_at, spf_at, auto_recovery_approval});
  }

  required_modules_map_.insert(std::make_pair(key, required_modules));

}


// Register diag and msg from message to map
void DiagnosticErrorMonitor::onDiagArray(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  diag_array_ = msg;
  
  const auto & header = msg->header;

  for (const auto & diag : msg->status) {
    if (diag_buffer_map_.count(diag.name) == 0) {
        diag_buffer_map_.insert(std::make_pair(diag.name,DiagBuffer{}));
      }

      auto & diag_buffer = diag_buffer_map_.at(diag.name);
      diag_buffer.push_back(DiagStamped{header, diag});

      while (diag_buffer.size() > diag_buffer_size_) {
      diag_buffer.pop_front();
    }
  }
}


//Timer functions
bool DiagnosticErrorMonitor::isDataReady()
{
  if (!diag_array_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for diag_array msg...");
    return false;
  }
  //TODO: Add any other message
  return true;
}

void DiagnosticErrorMonitor::onTimer()
{
  if (!isDataReady()) {
    if ((this->now() - initialized_time_).seconds() > params_.data_ready_timeout) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "input data is timeout");
      publishHazardStatus(createTimeoutHazardStatus());
    }
    return;
  }
  
  //TODO: Define config yaml keynames
  current_mode_ = KeyName::autonomous_driving;
  updateHazardStatus();
  publishHazardStatus(hazard_status_);
}

//get latest diag from map

std::optional<DiagStamped> DiagnosticErrorMonitor::getLatestDiag(
    const std::string & diag_name) const
{
  if (diag_buffer_map_.count(diag_name) == 0) {
    return {};
  }
  
  const auto & diag_buffer = diag_buffer_map_.at(diag_name);
  
  if (diag_buffer.empty()) {
    return {};
  }

  return diag_buffer.back();
}

uint8_t DiagnosticErrorMonitor::getHazardLevel(const DiagConfig & required_module, const int diag_level) const
{
  using watchdog_system_msgs::msg::HazardStatus;

  if (isOverLevel(diag_level, required_module.spf_at)) {
    return HazardStatus::SINGLE_POINT_FAULT;
  }
  if (isOverLevel(diag_level, required_module.spf_at)) {
    return HazardStatus::LATENT_FAULT;
  }
  if (isOverLevel(diag_level, required_module.spf_at)) {
    return HazardStatus::SAFE_FAULT;
  }

  return HazardStatus::NO_FAULT;
}

// Register DiagnosticStatus to HazardStatus by each hazard level
void DiagnosticErrorMonitor::appendHazardDiag(
    const DiagConfig & required_module, const diagnostic_msgs::msg::DiagnosticStatus & hazard_diag,
    watchdog_system_msgs::msg::HazardStatus * hazard_status) const
{
  const auto hazard_level = getHazardLevel(required_module, hazard_diag.level);

  // Get target diagnostic array ref
  auto & target_diagnostics_ref = getTargetDiagnosticsRef(hazard_level, hazard_status);
  // Register to ref
  target_diagnostics_ref.push_back(hazard_diag);

  // Register child DiagnosticStatus
  if (params_.add_leaf_diagnostics) {
    for (const auto & diag :
         diagnostics_filter::extractLeafChildrenDiagnostics(hazard_diag, diag_array_->status)) {
      target_diagnostics_ref.push_back(diag);
    }
  }

  hazard_status->level = std::max(hazard_status->level, hazard_level);
}

watchdog_system_msgs::msg::HazardStatus DiagnosticErrorMonitor::judgeHazardStatus() const
{
  using watchdog_system_msgs::msg::HazardStatus;
  using diagnostic_msgs::msg::DiagnosticStatus;

  HazardStatus hazard_status;
  
  for (const auto & required_module : required_modules_map_.at(current_mode_)) {
    const auto & diag_name = required_module.name;
    const auto latest_diag = getLatestDiag(diag_name);

    // no diag found
    if (!latest_diag) {
      if (!params_.ignore_missing_diagnostics) {
        DiagnosticStatus missing_diag;
        missing_diag.name = diag_name;
        missing_diag.hardware_id = "system_error_monitor";
        missing_diag.level = DiagnosticStatus::STALE;
        missing_diag.message = "no diag found";

        appendHazardDiag(required_module, missing_diag, &hazard_status);
      }

      continue;
    }

    // diag level high
    {
      appendHazardDiag(required_module, latest_diag->status, &hazard_status);
    }

    // diag timeout
    {
      const auto time_diff = this->now() - latest_diag->header.stamp;
      if (time_diff.seconds() > params_.diag_timeout_sec) {
        DiagnosticStatus timeout_diag = latest_diag->status;
        timeout_diag.level = DiagnosticStatus::STALE;
        timeout_diag.message = "timeout";

        appendHazardDiag(required_module, timeout_diag, &hazard_status);
      }
    }
  }

  return hazard_status;
}

void DiagnosticErrorMonitor::updateHazardStatus()
{
  //const bool prev_emergency_status = hazard_status_.emergency;

  // Create hazard status based on diagnostics
  if (!hazard_status_.emergency_holding) {
    const auto current_hazard_status = judgeHazardStatus();
    hazard_status_.level = current_hazard_status.level;
    hazard_status_.diag_no_fault= current_hazard_status.diag_no_fault;
    hazard_status_.diag_safe_fault= current_hazard_status.diag_safe_fault;
    hazard_status_.diag_latent_fault= current_hazard_status.diag_latent_fault;
    hazard_status_.diag_single_point_fault= current_hazard_status.diag_single_point_fault;
  }

  // Update emergency status
  {
    hazard_status_.emergency = hazard_status_.level >= params_.emergency_hazard_level;

    //Emergency hold related
    /*if (hazard_status_.emergency != prev_emergency_status) {
      emergency_state_swich_time_ = this->now();
    }*/
  }

  //TODO emergency_holding condition?
}

// Emergency hold related function
/*bool DiagnosticErrorMonitor::canAutoRecovery() const
{
  const auto error_modules = getErrorModules(hazard_status_, params_.emergency_hazard_level);

  for (const auto & required_module : required_modules_map_.at(current_mode_)) {
    if(required_module.auto_recovery) {
      continue;
    }
    // if cannot auto recovery
    if (error_modules.count(requred_module.name) != 0) {
      return false;
    }
  }
  return true;
}*/


void DiagnosticErrorMonitor::publishHazardStatus(
    const watchdog_system_msgs::msg::HazardStatus & hazard_status)
{
  watchdog_system_msgs::msg::HazardStatusStamped hazard_status_stamped;
  hazard_status_stamped.stamp = this->now();
  hazard_status_stamped.status= hazard_status;
  pub_hazard_status_->publish(hazard_status_stamped);
}
