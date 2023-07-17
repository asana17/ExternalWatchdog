#include "dummy_hazard_status_publisher/dummy_hazard_status_publisher.hpp"
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dummy_hazard_status_publisher::DummyHazardStatusPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
