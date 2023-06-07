#include "diagnostic_error_monitor/diagnostic_error_monitor_core.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiagnosticErrorMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
