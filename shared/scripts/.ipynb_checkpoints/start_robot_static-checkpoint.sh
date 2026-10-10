#!/bin/bash
# RosForge — Start Robot Static (Joint State GUI mode)
source /opt/ros/jazzy/setup.bash

# Kill any existing processes
echo "Cleaning up previous session..."
pkill -9 -f "topic pub" 2>/dev/null
pkill -9 -f robot_simulator.py 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f joint_state_publisher_gui 2>/dev/null
pkill -9 -f ros2_monitor.py 2>/dev/null
pkill -9 -f static_transform_publisher 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
sleep 2

# Start simulator briefly just to get robot_description
echo "Getting robot description..."
python3 /home/jovyan/course_materials/scripts/robot_simulator.py &
SIM_PID=$!
sleep 2

ros2 topic echo /robot_description --once --field data 2>/dev/null | \
python3 -c "
import sys, yaml
data = sys.stdin.read().replace('---','').strip()
open('/tmp/robot.urdf','w').write(data)
yaml.dump({'robot_state_publisher':{'ros__parameters':{'robot_description':data}}}, open('/tmp/rsp_params.yaml','w'))
"

pkill -9 -f robot_simulator.py 2>/dev/null
sleep 1

echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args --params-file /tmp/rsp_params.yaml &
RSP_PID=$!
sleep 2

echo "Starting static transform publisher..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint &
STP_PID=$!
sleep 1

echo "Starting Joint State Publisher GUI..."
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
JSP_PID=$!
sleep 1

echo "Starting RViz..."
ros2 run rviz2 rviz2 -d /home/jovyan/course_materials/config/rosforge.rviz

# When RViz closes, stop everything
pkill -9 -f "topic pub" 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f static_transform_publisher 2>/dev/null
pkill -9 -f joint_state_publisher_gui 2>/dev/null
