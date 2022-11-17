#include "tilde_aggregator/tilde_aggregator_core.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TildeAggregator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
