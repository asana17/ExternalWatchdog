# tilde_error_monitor

## easy test

1. sensing部にTILDEを適用したautowareを用意
1. lsimを起動
1. ros2 launch tilde_aggregator tilde_aggregator.launch.xml
1. ros2 launch tilde_error_monitor tilde_error_monitor.launch.xml
1. ros2 topic echo /watchdog/emergency/tilde_hazard_status
