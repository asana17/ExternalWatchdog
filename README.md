# Safety Island
Currently WIP.

This repository contains safety island components. `supervisor(_psim)`, `emergency_stop_operator` pkg is used for ecu switching MRM.

## pkg description
- `supervisor`, `supervisor_psim`

  Includes `voter` and `switch` functionality in one ROS2 node.

- `emergency_stop_operator`

  Operate MRM emergency stop on Supervisor ECU (can operate MRM independently).

- `diagnostic_monitor`

  Judge system `hazard_status` from ROS2 diagnostic msg.

- `tilde_monitor`

  Judge system `hazard_status` from TILDE deadline restrictions. (Currently pending.)

- `hazard_status_converter`

  Convert Autoware style hazard_status to safety_island style hazard_status.

- `dummy_hazard_status_publisher`

  Publish safety_island style dummy hazard_status.

- `launcher`

  Launch safety_island planning simulation.

- `supervisor_vehicle_cmd_gate`, `raw_vehicle_cmd_converter`

  Convert vehicle `control_cmd` to CAN frame (not implemented completly).

- `watchdog`

  Temporal external MRM operator. (not implemented completly.)


## How to build
  ```
   $ vcs import src < build_depends.repos
   $ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-skip tilde_error_monitor tilde_aggregator external_watchdog
  ```

## vehicle and psim ver.
Currently working on with planning simulator (`supervisor_psim` pkg). `supervisor` pkg is for real vehicle.

Please look at `supervisor_psim` directory to quick run.
