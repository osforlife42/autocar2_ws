#! /bin/bash

x=${1}
z=${2}
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{'linear': {'x': ${x}, 'y': 0,  'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': ${z}}}" 
