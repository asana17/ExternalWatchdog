#!/bin/bash

export AUTOWARE_DIR=~/pilot-auto.x2
source /opt/ros/humble/setup.bash
source $AUTOWARE_DIR/install/setup.bash

ros2 service call /api/operation_mode/change_to_autonomous autoware_adapi_v1_msgs/srv/ChangeOperationMode
