#!/bin/bash

source /opt/ros/humble/setup.bash

ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped\
 '{header: {frame_id: "map"}, pose: {pose: {position: {x: 3709.22, y: 73702.4, z: 0}, orientation: {x: 0, y: 0, z: 0.850777, w: 0.525527}}}}'

ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/PoseStamped\
 '{header: {frame_id: "map"}, pose: {position: {x: 3746.09, y: 73738.1, z: 0}, orientation: {x: 0, y: 0, z: -0.175571, w: 0.984467}}}'
