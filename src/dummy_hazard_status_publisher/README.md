# Dummy Hazard Status Pulbisher

## description
publish dummy hazard status to voter.

## dynamic param reconfigure
set following parameters: `'[level, emergency, emergency_holding, is_self_recoverable]'`
```
ros2 param set /dummy_hazard_status_publisher hazard_status_params '["0","true","true","true"]'
```

