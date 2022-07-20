#!/bin/bash

RUN_MACHINE=`jq ".components.machine" ${ROS2_DOCKER_WS}/system_scripts/system_setup.json`
RUN_MACHINE=$(echo $RUN_MACHINE | tr -d '""')

JETSON_MACHINE_IP=`jq ".components.jetson_machine_ip" ${ROS2_DOCKER_WS}/system_scripts/system_setup.json` 
JETSON_MACHINE_IP=$(echo $JETSON_MACHINE_IP | tr -d '""')

docker container kill dorothy_sim

if [ $RUN_MACHINE == "pc" ]; then 
    docker container kill dorothy_edge
fi

if [ $RUN_MACHINE == "jetson" ]; then 
    ssh user@${JETSON_MACHINE_IP} 'docker container kill dorothy_edge'
fi
