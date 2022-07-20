#!/bin/bash
xhost +

tmux new-session -d -s TEST_SESSION
tmux bind k 'display-message "killing"; send-keys kill_edge_sim.sh KPENTER; send-keys "sleep 1" KPENTER; '
tmux new-window  -n dorothy2_sim  
tmux select-window -t "dorothy2_sim"
tmux send-keys 'echo "HI" ' KPENTER

tmux new-window  -n sitl
tmux select-window -t "sitl" 
tmux send-keys 'echo "SITL" ' KPENTER

tmux att 