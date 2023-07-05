# supervisor_psim

## psim
to use psim,  subscribe/publish control command
### launch
1. remap following main psim launch parameter

  ```
  ("input/ackermann_control_command", "/control_switch_interface/control_cmd"),
  ("input/gear_command", "/control_switch_interface/gear_cmd"),
  ("input/turn_indicators_command", "/control_switch_interface/turn_indicators_cmd"),
  ("input/hazard_lights_command", "/control_switch_interface/hazard_lights_cmd"),
  ```

2.
