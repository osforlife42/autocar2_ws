#!/bin/bash
xhost +

tmux new-session -d -s DOROTHY_EDGE
RUN_MACHINE=`jq ".components.machine" ${ROS2_DOCKER_WS}/system_scripts/system_setup.json`
RUN_MACHINE=$(echo $RUN_MACHINE | tr -d '""')

JETSON_MACHINE_IP=`jq ".components.jetson_machine_ip" ${ROS2_DOCKER_WS}/system_scripts/system_setup.json` 
JETSON_MACHINE_IP=$(echo $JETSON_MACHINE_IP | tr -d '""')

EDGE_DOMAIN_ID=`jq ".domain_ids.edge_domain_id" ${ROS2_DOCKER_WS}/system_scripts/system_setup.json`
EDGE_DOMAIN_ID=$(echo $EDGE_DOMAIN_ID | tr -d '""') 
tmux bind k 'display-message "killing"; send-keys kill_sim.sh KPENTER; send-keys "sleep 1" KPENTER;'

kill_sim.sh

# EDGE_DOMAIN_ID=11
tmux new-window  -n dorothy2_sim  
tmux select-window -t "dorothy2_sim"
tmux send-keys "start_dorothy_sim.sh $EDGE_DOMAIN_ID" KPENTER


if [ $RUN_MACHINE == "pc" ]; then 
    tmux new-window  -n dorothy_edge_pc
    tmux select-window -t "dorothy_edge_pc"
    tmux send-keys "start_dorothy_edge.sh $EDGE_DOMAIN_ID" KPENTER
fi
if [ $RUN_MACHINE == "jetson" ]; then 
    tmux new-window  -n dorothy_edge_jetson
    tmux select-window -t "dorothy_edge_jetson"
    tmux send-keys "ssh user@${JETSON_MACHINE_IP} 'cd ~/projects/dorothy2_ws/system_scripts/setup; source ./setup_host_env.sh; start_dorothy_edge.sh $EDGE_DOMAIN_ID'" KPENTER
fi
# 
# tmux new-window  -n sitl_with_gazebo  
# tmux select-window -t "sitl_with_gazebo"
# tmux send-keys 'run_sitl.sh' KPENTER

tmux new-window  -n free_terminal
tmux select-window -t "free_terminal"
tmux send-keys 'echo "to kill all edge dockers press: Ctrl+a, k" ' KPENTER
tmux send-keys 'echo "running edge nodes on machine: '${RUN_MACHINE}'" ' KPENTER

tmux att 