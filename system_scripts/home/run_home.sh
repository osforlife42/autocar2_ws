#!/bin/bash
xhost +

tmux new-session -d -s DOROTHY_HOME

tmux bind h 'display-message "killing"; send-keys kill_home.sh KPENTER; send-keys "sleep 1" KPENTER;'

HOME_DOMAIN_ID=`jq ".domain_ids.home_domain_id" ${ROS2_DOCKER_WS}/system_scripts/system_setup.json`
HOME_DOMAIN_ID=$(echo $HOME_DOMAIN_ID | tr -d '""') 

EDGE_DOMAIN_ID=`jq ".domain_ids.edge_domain_id" ${ROS2_DOCKER_WS}/system_scripts/system_setup.json`
EDGE_DOMAIN_ID=$(echo $EDGE_DOMAIN_ID | tr -d '""') 

kill_home.sh

tmux new-window  -n rviz
tmux select-window -t "rviz"
tmux send-keys "start_dorothy_ui.sh $HOME_DOMAIN_ID" KPENTER

tmux new-window  -n point_and_shoot
tmux select-window -t "point_and_shoot"
tmux send-keys "run_point_and_shoot.sh $HOME_DOMAIN_ID" KPENTER

tmux new-window  -n domain_bridge
tmux select-window -t "domain_bridge"
tmux send-keys "start_domain_bridge.sh $EDGE_DOMAIN_ID $HOME_DOMAIN_ID " KPENTER

# tmux new-window  -n sitl_with_gazebo  
# tmux select-window -t "sitl_with_gazebo"
# tmux send-keys 'run_sitl.sh' KPENTER





tmux new-window  -n free_terminal
tmux select-window -t "free_terminal"
tmux send-keys 'echo "to kill all home dockers press: Ctrl+a, h" ' KPENTER

tmux att 