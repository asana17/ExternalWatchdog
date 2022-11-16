#ifndef TILDE_AGGREGATOR__TILDE_AGGREGATOR_CORE_HPP_
#define TILDE_AGGREGATOR__TILDE_AGGREGATOR_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <unordered_map>
#include <deque>


#include "tilde_msg/msg/message_tracking_tag.hpp"
#include "watchdog_system_msgs/msg/tilde_diagnostic_array.hpp"


using RequiredTopics = std::vector<std::string>;

struct TildePathConfig
{
  std::string end_point;
  std::string start_point;
  rclcpp::Duration deadline;
  std::string level;
};

using RequiredPaths = std::vector<TildePathConfig>;


/*struct PubTopicTimeInfoStamped
{
  std_msgs::msg::Header header;
  tilde_msg::msg::PubTopicTimeInfo pub_topic_time_info;
};

using PubTopicTimeInfoBuffer = std::deque<PubTopicTimeInfoStamped>;

struct SubTopicTimeInfoStamped
{
  std_msgs::msg::Header header;
  tilde_msg::msg::SubTopicTimeInfo sub_topic_time_info;
};*/

using SubTopicTimeInfoBuffer = std::vector<tilde_msg::msg::SubTopicTimeInfo>;


struct MessageTrackingTagStamped
{
  std_msgs::msg::Header header;
  tilde_msg::msg::PubTopicTimeInfo pub_topic_time_info;
  SubTopicTimeInfoBuffer sub_topic_time_infos;
};

using MessageTrackingTagBuffer = std::deque<MessageTrackingTagStamped>;

struct KeyName
{
  static constexpr const char * autonomous_driving = "autonomous_driving";
  static constexpr const char * test= "test";
  static constexpr const char * test_sensing= "test_sensing";
};

class TildeAggregator : public rclcpp::Node
{
public:
  TildeAggregator();

private:

  struct Parameters
  {
    int update_rate;
    double message_tracking_tag_timeout_sec;
    double data_ready_timeout;
  };

  Parameters params_{};

  rclcpp::Time initialized_time_;
  watchdog_system_msgs::msg::TildeDiagnosticArray tilde_diagnostic_array_{};
  std::unordered_map<std::string, RequiredTopics> required_topics_map_;
  std::unordered_map<std::string, RequiredPaths> required_paths_map_;
  std::string current_mode_;

  void loadRequiredTopics(const std::string & key);
  void loadRequiredPaths(const std::string & key);

  //Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  void onTimer();

  //Subscriber
  tilde_msg::msg::MessageTrackingTag::ConstSharedPtr message_tracking_tag_;
  void onMessageTrackingTag(
      const tilde_msg::msg::MessageTrackingTag::ConstSharedPtr msg);

  const size_t message_tracking_tag_buffer_size_ = 100;
  std::unordered_map<std::string, MessageTrackingTagBuffer> message_tracking_tag_buffer_map_;
  //std::unordered_map<std::string, SubTopicTimeInfoBuffer> sub_topic_time_info_buffer_map_;


  //Publisher
  rclcpp::Publisher<watchdog_system_msgs::msg::TildeDiagnosticArray>::SharedPtr
    pub_tilde_diagnostic_;
  void publishTildeDiag(watchdog_system_msgs::msg::TildeDiagnosticArray & tilde_diagnostic_array);


  //get tilde diag from message tracking tag
  std::optional<MessageTrackingTagStamped> getLatestMessageTrackingTag(const std::string & end_topic_name) const;
  std::optional<MessageTrackingTagStamped> getMessageTrackingTag(const std::string & topic_name, const rclcpp::Time header_stamp) const;
  void appendTildeDiagnosticStatus(
      const watchdog_system_msgs::msg::TildeDiagnosticStatus & tilde_diagnostic_status, watchdog_system_msgs::msg::TildeDiagnosticArray* tilde_diagnostic_array) const;
  uint8_t getTildeDiagLevel(const TildePathConfig & required_path, const MessageTrackingTagStamped & message_tracking_tag) const;
  std::optional<rclcpp::Duration> calculateResponseTime(const TildePathConfig & required_path, const MessageTrackingTagStamped & message_tracking_tag) const;
  std::optional<MessageTrackingTagStamped> findStartPoint(const TildePathConfig & required_path, const SubTopicTimeInfoBuffer & input_infos) const;
  watchdog_system_msgs::msg::TildeDiagnosticArray judgeTildeDiagnosticStatus() const;
  void updateTildeDiag();

};

#endif //TILDE_AGGREGATOR__TILDE_AGGREGATOR_CORE_HPP_
