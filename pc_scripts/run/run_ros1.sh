#!/bin/bash 
xhost +
docker run  --privileged --network host --rm -it  --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
ros1:noetic-test 