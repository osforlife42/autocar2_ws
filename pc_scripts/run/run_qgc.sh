#! /bin/bash
xhost +
docker run  --network host --rm -it --env="DISPLAY" --privileged --env="QT_X11_NO_MITSHM=1" \
--volume="/dev:/dev" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  \
--volume="$ROS2_DOCKER_WS/sitl/configs:/home/user/.config/:rw" -e QT_QUICK_BACKEND=software \
--name qgc ardupilot:new /bin/bash -c -i 'QGroundControl.AppImage'


