#!/bin/bash
source /opt/ros/humble/setup.bash

ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode
