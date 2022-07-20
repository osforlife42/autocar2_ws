#! /bin/bash

docker run --network host --rm -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
--volume="/dev:/dev" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  \
--volume="$ROS2_DOCKER_WS/sitl/configs:/home/user/.config/:rw" \
--name sitl_with_gazebo \
ardupilot:new bash -i -c 'sim_vehicle.py -v Rover -f gazebo-rover -I0 --add-param-file=/home/user/.config/pix_params.parm -m '--mav10 -l 33.3,33.3,0,0 --cmd="set streamrate 50;set streamrate2 50"' '

