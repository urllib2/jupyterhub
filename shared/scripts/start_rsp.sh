#!/bin/bash
source /opt/ros/jazzy/setup.bash

echo "Waiting for robot_description..."
ros2 topic echo /robot_description --once --field data 2>/dev/null > /tmp/robot.urdf

echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat /tmp/robot.urdf)"
