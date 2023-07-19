# Dummy Hazard Status Pulbisher

## Description
Publish dummy hazard status to voter.

## Dynamic Parameter Reconfiguration
Set following parameters: `'[level, emergency, emergency_holding, is_self_recoverable]'`.

`TOPIC_NAME` must be a name that exist in the parameter YAML file.
```
ros2 param set /dummy_hazard_status_publisher TOPIC_NAME.hazard_status_params '["0","true","true","true"]'
```

