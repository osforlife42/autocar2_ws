#! /bin/bash
# build all the dockers in the this order (in the future it will be replaced with only the last build 
# and a good registry)
docker build -t dorothy2-edge:nvidia-focal-base $ROS2_DOCKER_WS/edge/jetson_base
docker build -t dorothy2-edge:galactic-base $ROS2_DOCKER_WS/edge/galactic_jetson_base
docker build -t dorothy2-edge:dev $ROS2_DOCKER_WS/edge
