#!/bin/bash 
xhost +
docker run  --privileged --network host --rm -it  --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
--volume="/dev:/dev" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  \
--volume="$ROS2_DOCKER_WS/scripts:/home/ros/dev_ws/scripts:rw" --volume="$ROS2_DOCKER_WS/docker_save:/home/ros/dev_ws/docker_save:rw" \
--user "$(id -u):$(id -g)" --name "dorothy_ui" dorothy2-ui:dev # \
# /bin/bash -c -i 'ros2 run rqt_py_console rqt_py_console' 

