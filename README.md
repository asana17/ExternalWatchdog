# Safety Island
Currently WIP.

This repository contains safety island components. The `supervisor(_psim)`, `emergency_stop_operator` package are mainly used for safety island funcitons.

## Package Descriptions
- `supervisor`, `supervisor_psim`

  Includes `voter` and `switch` functionality in one ROS2 node.

- `emergency_stop_operator`

  Operate MRM emergency stop on Supervisor ECU (can operate MRM independently).

- `diagnostic_monitor`

  Judge system `hazard_status` from ROS2 diagnostic msgs.

- `tilde_monitor`

  Judge system `hazard_status` from TILDE deadline restrictions. (Currently pending.)

- `hazard_status_converter`

  Convert Autoware style `hazard_status` to safety_island style `hazard_status`.

- `dummy_hazard_status_publisher`

  Publish safety_island style dummy `hazard_status`.

- `launcher`

  Launch safety_island planning simulation.

- `supervisor_vehicle_cmd_gate`, `raw_vehicle_cmd_converter`

  Convert vehicle `control_cmd` to CAN frames (not implemented completly).

- `watchdog`

  A temporal external MRM operator. (Not implemented completly and currently pending.)


## How to Build
  ```
   $ vcs import src < build_depends.repos
   $ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-skip tilde_error_monitor tilde_aggregator external_watchdog
  ```

## How to Run
### Vehicle and Planning Simulator ver.
Currently working on planning simulation (The `supervisor_psim` package). The `supervisor` pacakge is for real vehicle implementation.

Please look at the `supervisor_psim` directory for a quick run.
