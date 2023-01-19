#include "external_watchdog/external_watchdog_core.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExternalWatchdog>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
