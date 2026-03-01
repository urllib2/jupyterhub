#!/bin/bash
# RosForge — Start Robot System
# Starts everything: simulator + robot_state_publisher + RViz
source /opt/ros/jazzy/setup.bash

echo "Starting robot simulator..."
python3 /home/jovyan/course_materials/scripts/robot_simulator.py &
SIM_PID=$!
sleep 2

echo "Starting robot_state_publisher..."
ros2 topic echo /robot_description --once --field data 2>/dev/null | \
python3 -c "
import sys, yaml
data = sys.stdin.read().replace('---','').strip()
open('/tmp/robot.urdf','w').write(data)
yaml.dump({'robot_state_publisher':{'ros__parameters':{'robot_description':data}}}, open('/tmp/rsp_params.yaml','w'))
"
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file /tmp/rsp_params.yaml &
RSP_PID=$!
sleep 2

echo "Starting RViz..."
ros2 run rviz2 rviz2 -d /home/jovyan/course_materials/config/rosforge.rviz

# When RViz closes, stop everything
kill $SIM_PID $RSP_PID 2>/dev/null
