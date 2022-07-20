#!/bin/bash 

# create empty folders in the workspace directory as git doesn't track those 
# this fails the first build (cmakelists.txt looks for these folders )
# in order to fix this, in the docker building process create those empty folders
# and then connect them to your workspace as a volume 
 
empty_folders="config launch maps meshes models params rviz src worlds"
packages="dorothy_robot basic_mobile_robot"
for package in $packages; do
    for folder in $empty_folders; do 
        mkdir -p $ROS2_DOCKER_WS/src/$package/$folder
    done
done

empty_ws_folders="install docker_build"
for folder in $empty_ws_folders; do 
    mkdir -p $ROS2_DOCKER_WS/$folder
done