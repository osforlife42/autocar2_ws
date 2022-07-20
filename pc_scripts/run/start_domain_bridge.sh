#!/bin/bash 
xhost +

# setup domain ids  - TBD later ! :
# if  [ -z "$1" ]; then ROS_DOMAIN_ID=$ROS_DEFAULT_DOMAIN_ID; else ROS_DOMAIN_ID=$1; fi
EDGE_DOMAIN_ID=$1
HOME_DOMAIN_ID=$2

docker run  --privileged --network host --rm -it  --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
--volume="/dev:/dev" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  \
--volume="$ROS2_DOCKER_WS/src:/home/ros/dev_ws/src:rw" --volume="$ROS2_DOCKER_WS/install:/home/ros/dev_ws/install:rw" \
--volume="$ROS2_DOCKER_WS/scripts:/home/ros/dev_ws/scripts:rw" --volume="$ROS2_DOCKER_WS/docker_save:/home/ros/dev_ws/docker_save:rw" \
--volume="$ROS2_DOCKER_WS/docker_build:/home/ros/dev_ws/build:rw" \
--user "$(id -u):$(id -g)" --name "domain_bridge" dorothy2-ui:dev  \
/bin/bash -c -i "ros2 run domain_bridge domain_bridge --from $EDGE_DOMAIN_ID --to $HOME_DOMAIN_ID /home/ros/dev_ws/src/dorothy_robot/config/domain_bridge_config.yaml "
