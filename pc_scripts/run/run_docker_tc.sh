#! /bin/bash
xhost +
# docker network create test-net 
docker run  --network host --rm -it --env="DISPLAY" --privileged --env="QT_X11_NO_MITSHM=1" \
--volume="/dev:/dev" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  \
--cap-add NET_ADMIN \ 
# --volume="/var/docker-tc:/var/docker-tc:ro" \
--volume="/var/run/docker.sock:/var/run/docker.sock"  --name docker-tc  lukaszlach/docker-tc


