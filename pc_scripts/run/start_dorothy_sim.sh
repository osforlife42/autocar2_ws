#!/bin/bash 
xhost +

# setup ros domain id:
if  [ -z "$1" ]; then ROS_DOMAIN_ID=$ROS_DEFAULT_DOMAIN_ID; else ROS_DOMAIN_ID=$1; fi

docker run  --privileged --network host --rm -it  --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --env="ROS_DOMAIN_ID=$ROS_DOMAIN_ID" \
--volume="/dev:/dev" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  \
--volume="$ROS2_DOCKER_WS/src:/home/ros/dev_ws/src:rw" --volume="$ROS2_DOCKER_WS/install:/home/ros/dev_ws/install:rw" \
--volume="$ROS2_DOCKER_WS/scripts:/home/ros/dev_ws/scripts:rw" --volume="$ROS2_DOCKER_WS/docker_save:/home/ros/dev_ws/docker_save:rw" \
--volume="$ROS2_DOCKER_WS/docker_build:/home/ros/dev_ws/build:rw" \
--user "$(id -u):$(id -g)" --name "dorothy_sim" dorothy2-sim:dev \
/bin/bash -c -i 'ros2 launch dorothy_robot dorothy_only_sim.launch.py ' 

# --privileged 
# --name="nav2_docker" 
# -v $CAR_HOME:/car
