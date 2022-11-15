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

#include <string>
#include <vector>
#include <regex>

#include "tilde_aggregator/tilde_aggregator_core.hpp"
#include "tilde_msg/msg/message_tracking_tag.hpp"


#define FMT_HEADER_ONLY
#include <fmt/format.h>


namespace{

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
  using watchdog_system_msgs::msg::TildeDiagnosticStatus;
  using std::regex_constants::icase;

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

#define INPUT_DATA_TIMEOUT_ERROR 2

watchdog_system_msgs::msg::TildeDiagnosticArray createTimeoutTildeDiagnosticArray()
{
  watchdog_system_msgs::msg::TildeDiagnosticArray tilde_diagnostic_array;
  watchdog_system_msgs::msg::TildeDiagnosticStatus tilde_diagnostic_status;
  auto tilde_diagnostic_status_ref = tilde_diagnostic_array.status;
  tilde_diagnostic_status.level = INPUT_DATA_TIMEOUT_ERROR;
  tilde_diagnostic_status.start_point = "";
  tilde_diagnostic_status.end_point = "";
  tilde_diagnostic_status.message = "input data timeout";

  tilde_diagnostic_status_ref.push_back(tilde_diagnostic_status);
  return tilde_diagnostic_array;
}

}//namespace




TildeAggregator::TildeAggregator()
  : Node(
      "tilde_aggregator",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{

  //Parameters
  get_parameter_or<int>("update_rate", params_.update_rate, 10);
  get_parameter_or<double>("message_tracking_tag_timeout_sec", params_.message_tracking_tag_timeout_sec, 1.0);
  get_parameter_or<double>("data_ready_timeout", params_.data_ready_timeout, 30.0);

  //load topics and paths
  loadRequiredTopics(KeyName::test_sensing);
  loadRequiredPaths(KeyName::test_sensing);

  using std::placeholders::_1;

  //Subscriber
  for (const auto & required_topic : required_topics_map_.at(KeyName::test_sensing)) {

    //TODO error handling when no topic
    create_subscription<tilde_msg::msg::MessageTrackingTag>(
        required_topic, rclcpp::QoS{1}, std::bind(&TildeAggregator::onMessageTrackingTag, this, _1));
  }

  //Publisher
  pub_tilde_diagnostic_ = create_publisher<watchdog_system_msgs::msg::TildeDiagnosticArray> (
      "~/output/tilde_agg", rclcpp::QoS{1});

  //Timer
  initialized_time_ = this->now();
  const auto period_ns = rclcpp::Rate(params_.update_rate).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&TildeAggregator::onTimer, this));

}



//load required topics from config yaml
void TildeAggregator::loadRequiredTopics(const std::string & key)
{
  const auto param_key = std::string("required_topics.") + key;

  const uint64_t depth = 2;
  const auto param_names = this->list_parameters({"required_topics"}, depth).names;
  if (param_names.empty()) {
    throw std::runtime_error(fmt::format("no parameter found: {}", param_key));
  }


  std::set<std::string> topic_names;
  RequiredTopics required_topics;

  for (const auto & param_name: param_names) {
    //Example of param_name: required_topics.topic_name
    const auto split_names = split(param_name, '.');
    const auto & param_required_topics = split_names.at(0);
    const auto & param_key = split_names.at(1);
    const auto & param_topic_name = split_names.at(2);


    const auto & topic_name_with_prefix =
      fmt::format("{0}.{1}.{2}", param_required_topics, param_key, param_topic_name);

    if (topic_names.count(topic_name_with_prefix) != 0) {
      continue; //Skip duprecated topic
    }

    //Register name
    topic_names.insert(topic_name_with_prefix);

    //Register topics

    const auto param_topic_name_with_prefix = param_topic_name + std::string("/message_tracking_tag");
    required_topics.push_back(param_topic_name_with_prefix);
  }

  required_topics_map_.insert(std::make_pair(key,required_topics));
}

// load required paths and params from config yaml
void TildeAggregator::loadRequiredPaths(const std::string & key)
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

  for (const auto & param_name: param_names) {
    // Example of param_name: required_paths.key.start_point.end_point
    //                    or  required_paths.key.start_point.end_point.parameter
    const auto split_names = split(param_name, '.');
    const auto & param_required_paths = split_names.at(0);
    const auto & param_key = split_names.at(1);
    const auto & param_end_point = split_names.at(2);
    const auto & param_start_point = split_names.at(3);

    const auto & path_name_with_prefix =
      fmt::format("{0}.{1}.{2}.{3}", param_required_paths, param_key, param_end_point, param_start_point);

    if (path_names.count(path_name_with_prefix) != 0) {
      continue; //Skip duprecated path
    }

    //Register name
    path_names.insert(path_name_with_prefix);

    const auto deadline_key = path_name_with_prefix + std::string(".deadline");
    std::string deadline;
    this->get_parameter_or(deadline_key, deadline, std::string("none"));
    const auto split_string_deadline = split(deadline, '.');
    if (split_string_deadline.size() != 2) {
      throw std::runtime_error(fmt::format("deadline format error: {}", param_key));
    }
    int32_t deadline_sec = static_cast<int32_t>(std::stol(split_string_deadline[0]));
    uint32_t deadline_nsec = static_cast<uint32_t>(std::stoul(split_string_deadline[1]));
    const auto time_deadline = rclcpp::Duration(deadline_sec, deadline_nsec);

    const auto level_key = path_name_with_prefix + std::string(".level");
    std::string level;
    this->get_parameter_or(level_key, level, std::string("none"));

    //Register each path
    required_paths.push_back({param_end_point, param_start_point, time_deadline, level});
  }

  required_paths_map_.insert(std::make_pair(key, required_paths));

}

//register message tracking tag to map
void TildeAggregator::onMessageTrackingTag(
    const tilde_msg::msg::MessageTrackingTag::ConstSharedPtr msg)
{
  message_tracking_tag_ = msg;

  const auto & header = msg->header;
  const auto & output_info = msg->output_info;

  const auto & topic_name = output_info.topic_name;

  if (message_tracking_tag_buffer_map_.count(topic_name) == 0) {
    message_tracking_tag_buffer_map_.insert(std::make_pair(topic_name, MessageTrackingTagBuffer{}));
  }
  auto & message_tracking_tag_buffer = message_tracking_tag_buffer_map_.at(topic_name);

  MessageTrackingTagStamped message_tracking_tag_stamped;

  message_tracking_tag_stamped.header = header;
  message_tracking_tag_stamped.pub_topic_time_info = output_info;
  for (const auto & input_info : msg->input_infos) {
    message_tracking_tag_stamped.sub_topic_time_infos.push_back(input_info);
  }

  message_tracking_tag_buffer.push_back(message_tracking_tag_stamped);
  while (message_tracking_tag_buffer.size() > message_tracking_tag_buffer_size_) {
     message_tracking_tag_buffer.pop_front();
  }
}

//Timer functions
bool TildeAggregator::isDataReady()
{
  if (!message_tracking_tag_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for message_tracking_tag...");
    return false;
  }
  return true;
}

void TildeAggregator::onTimer()
{
  if (!isDataReady()) {
    if ((this->now() - initialized_time_).seconds() > params_.data_ready_timeout) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), std::chrono::milliseconds(1000).count(), "input data is timeout");
      auto timeout_tilde_diagnostic_array = createTimeoutTildeDiagnosticArray();
      publishTildeDiag(timeout_tilde_diagnostic_array);
    }
    return;
  }
  current_mode_ = KeyName::test_sensing;
  updateTildeDiag();
  publishTildeDiag(tilde_diagnostic_array_);
}

//get latest message tracking tag from map
  std::optional<MessageTrackingTagStamped> TildeAggregator::getLatestMessageTrackingTag(const std::string & end_topic_name) const
{
  if (message_tracking_tag_buffer_map_.count(end_topic_name) == 0) {
    return {};
  }

  const auto & message_tracking_tag_buffer = message_tracking_tag_buffer_map_.at(end_topic_name);

  if (message_tracking_tag_buffer.empty()) {
    return {};
  }

  return message_tracking_tag_buffer.back();
}

#define RESPONSE_TIME_CALC_ERROR 4

uint8_t TildeAggregator::getTildeDiagLevel(const TildePathConfig & required_path, const MessageTrackingTagStamped & message_tracking_tag) const
{
  using watchdog_system_msgs::msg::TildeDiagnosticStatus;

  const auto deadline = required_path.deadline;

  const auto response_time = calculateResponseTime(required_path, message_tracking_tag);

  if (!response_time) {
    return RESPONSE_TIME_CALC_ERROR;
  }

  if (*response_time > deadline) {
    return str2level(required_path.level);
  }

  return TildeDiagnosticStatus::OK;

}

std::optional<rclcpp::Duration> TildeAggregator::calculateResponseTime(const TildePathConfig & required_path, const MessageTrackingTagStamped & message_tracking_tag) const
{
  rclcpp::Time pub_time = message_tracking_tag.pub_topic_time_info.pub_time;

  const auto & input_message_tracking_tag = findStartPoint(required_path, message_tracking_tag.sub_topic_time_infos);

  if (!input_message_tracking_tag) {
    return {};
  }

  rclcpp::Time initial_sub_time = input_message_tracking_tag->pub_topic_time_info.pub_time;

  const auto response_time = pub_time - initial_sub_time;

  return response_time;
}

std::optional<MessageTrackingTagStamped> TildeAggregator::findStartPoint(const TildePathConfig & required_path, const SubTopicTimeInfoBuffer & input_infos) const
{
  if (sizeof(input_infos) == 0) {
    return {};
  }
  const auto start_point = required_path.start_point;
  for (const auto & input_info : input_infos) {
    if (!input_info.has_header_stamp) {
      continue;
    }
    const auto input_message_tracking_tag = getMessageTrackingTag(input_info.topic_name, input_info.header_stamp);
    if (!input_message_tracking_tag) {
      continue;
    }
    if (input_info.topic_name == start_point) {
      return input_message_tracking_tag;
    }
    findStartPoint(required_path, input_message_tracking_tag->sub_topic_time_infos);
  }
  return {};
}

std::optional<MessageTrackingTagStamped> TildeAggregator::getMessageTrackingTag(const std::string & topic_name, const rclcpp::Time header_stamp) const
{
  const auto message_tracking_tag_buffer = message_tracking_tag_buffer_map_.at(topic_name);

  for (auto ritr = message_tracking_tag_buffer.rbegin(); ritr != message_tracking_tag_buffer.rend(); ++ritr) {
    const auto tag_header_stamp = ritr->header.stamp;
    if (tag_header_stamp == header_stamp) {
      return *ritr;
    }
  }
  return {};
}

//Register DiagnosticStatus

void TildeAggregator::appendTildeDiagnosticStatus (const watchdog_system_msgs::msg::TildeDiagnosticStatus & tilde_diagnostic_status, watchdog_system_msgs::msg::TildeDiagnosticArray * tilde_diagnostic_array) const {
  auto & target_tilde_diagnostic_status_ref = tilde_diagnostic_array->status;
  target_tilde_diagnostic_status_ref.push_back(tilde_diagnostic_status);
}

watchdog_system_msgs::msg::TildeDiagnosticArray TildeAggregator::judgeTildeDiagnosticStatus() const {

  using watchdog_system_msgs::msg::TildeDiagnosticArray;
  using watchdog_system_msgs::msg::TildeDiagnosticStatus;

  TildeDiagnosticArray tilde_diagnostic_array;

  for (const auto & required_path : required_paths_map_.at(current_mode_)) {

    const auto & end_point = required_path.end_point;
    const auto & start_point = required_path.start_point;
    const auto latest_message_tracking_tag = getLatestMessageTrackingTag(end_point);

    // no MessageTrackingTag found
    if (!latest_message_tracking_tag) {
      TildeDiagnosticStatus missing_tilde_diagnostic_status;
      missing_tilde_diagnostic_status.level = TildeDiagnosticStatus::ERROR;
      missing_tilde_diagnostic_status.start_point = start_point;
      missing_tilde_diagnostic_status.end_point = end_point;
      missing_tilde_diagnostic_status.message = "no message tracking tag found";

      appendTildeDiagnosticStatus(missing_tilde_diagnostic_status, &tilde_diagnostic_array);
    }
    
    {
      const auto tilde_diag_level = getTildeDiagLevel(required_path, *latest_message_tracking_tag);
      TildeDiagnosticStatus tilde_diagnostic_status;

      if (tilde_diag_level == RESPONSE_TIME_CALC_ERROR) {
        tilde_diagnostic_status.level = TildeDiagnosticStatus::WARN;
        tilde_diagnostic_status.message = "response time calc error";
      } else {
        tilde_diagnostic_status.level = tilde_diag_level;
        tilde_diagnostic_status.message = "";
      }

      tilde_diagnostic_status.start_point = start_point;
      tilde_diagnostic_status.end_point = end_point;

      appendTildeDiagnosticStatus(tilde_diagnostic_status, &tilde_diagnostic_array);
    }

    //diag timeout
    {
      const auto time_diff = this->now() - latest_message_tracking_tag->header.stamp;
      if (time_diff.seconds() > params_.message_tracking_tag_timeout_sec) {
        TildeDiagnosticStatus timeout_tilde_diagnostic_status;
        timeout_tilde_diagnostic_status.level = TildeDiagnosticStatus::ERROR;
        timeout_tilde_diagnostic_status.message = "timeout";
        timeout_tilde_diagnostic_status.start_point = start_point;
        timeout_tilde_diagnostic_status.end_point = end_point;

        appendTildeDiagnosticStatus(timeout_tilde_diagnostic_status, & tilde_diagnostic_array);

      }
    }
  }

  return tilde_diagnostic_array;

}


void TildeAggregator::updateTildeDiag()
{
  const auto current_tilde_diagnostic_array= judgeTildeDiagnosticStatus();
  tilde_diagnostic_array_.status= current_tilde_diagnostic_array.status;
}

void TildeAggregator::publishTildeDiag(watchdog_system_msgs::msg::TildeDiagnosticArray & tilde_diagnostic_array)
{
  tilde_diagnostic_array.header.stamp = this->now();
  pub_tilde_diagnostic_->publish(tilde_diagnostic_array);
}
