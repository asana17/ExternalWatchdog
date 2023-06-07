#ifndef VEHICLE_CMD_GATE_HPP_
#define VEHICLE_CMD_GATE_HPP_

#include <rclcpp/rclcpp.hpp>


namespace vehicle_cmd_gate
{


class VehicleCmdGate : public rclcpp::Node
{
public:
  explicit VehicleCmdGate(const rclcpp::NodeOptions & node_options);

};

}
#endif  // VEHICLE_CMD_GATE_HPP_
