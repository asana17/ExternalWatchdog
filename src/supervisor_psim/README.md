# supervisor_psim

safety island simulation using planning simulator.

## single ECU simulation
Simulation can run with only one ECU by using exclusive launch file.

Please look at `launcher` directory.

## How to run

### Preparation
#### Change planning simulator launch parameters
Remap following `simple_planning_simulator` launch parameter in Main ECU.

  ```
  ("input/ackermann_control_command", "/control_switch_interface/control_cmd"),
  ("input/gear_command", "/control_switch_interface/gear_cmd"),
  ("input/turn_indicators_command", "/control_switch_interface/turn_indicators_cmd"),
  ("input/hazard_lights_command", "/control_switch_interface/hazard_lights_cmd"),
  ```

#### Change SubECU launch parameters
Remap SubECU topic parameters to distinguish with that of Main ECU.

When operating single ECU simulation, this part can be skipped.


#### Build pilot-auto.x2 and safety_island
##### pilot-auto.x2
Please look at [Reference](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

##### safety_island

```
 $ vcs import src < build_depends.repos
 $ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --packages-skip tilde_error_monitor tilde_aggregator external_watchdog
```

### Run
#### Launch planning simulation and safety_island
It is better to launch safety_island first.

##### planning simulation
```
 $ ros2 launch autoware_launch planning_simulator.launch.xml map_path:=/home/asana/autoware_map/sample-map-planning vehicle_model:=gsm8 sensor_model:=aip_x2
```
##### safety island
```
 $ ros2 launch launcher safety_island_psim.launch.xml
```
For single ECU simulation,
```
 $ ros2 launch launcher safety_island_psim_single_autoware_ecu.launch.xml
```
