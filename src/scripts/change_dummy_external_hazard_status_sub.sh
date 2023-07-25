#!/bin/bash
source /opt/ros/humble/setup.bash

ros2 param set /dummy_hazard_status_publisher /main/external_monitoring/hazard_status/sub.hazard_status_params '["3","true","false","true"]'

ros2 param set /dummy_hazard_status_publisher /supervisor/external_monitoring/hazard_status/sub.hazard_status_params '["3","true","false","false"]'
