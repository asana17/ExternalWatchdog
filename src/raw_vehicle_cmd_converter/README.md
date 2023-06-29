# raw_vehicle_cmd_converter

`raw_vehicle_command_converter` is a node that converts desired acceleration and velocity to mechanical input for Supervisor ECU.

## Input topics

| Name                  | Type                                                     | Description                                                                                                        |
| --------------------- | -------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ |
| `~/input/control_cmd` | autoware_auto_control_msgs::msg::AckermannControlCommand | target `velocity/acceleration/steering_angle/steering_angle_velocity` is necessary to calculate actuation command. |

## Output topics

| Name                     | Type                                             | Description                                             |
| ------------------------ | ------------------------------------------------ | ------------------------------------------------------- |
| `~/output/actuation_cmd` | tier4_vehicle_msgs::msg::ActuationCommandStamped | actuation command for vehicle to apply mechanical input |
