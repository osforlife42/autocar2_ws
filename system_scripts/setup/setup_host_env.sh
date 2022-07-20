#! /bin/bash
THIS_SCRIPT_PATH=`realpath "$BASH_SOURCE"`
SCRIPTS_PATH="$(dirname $(dirname "$THIS_SCRIPT_PATH"))"
ROS2_DOCKER_WS="$(dirname "$SCRIPTS_PATH")"
echo "ROS2_DOCKER_WS IS NOW: $ROS2_DOCKER_WS"
export ROS2_DOCKER_WS

# detect architecture to determine jetson or pc 
arch_name=`arch`
if [[ $arch_name == x86* ]]; then 
	echo "you're running on pc!" 
	RUNNING_ON_JETSON=$false
	export RUNNING_ON_JETSON
	SCRIPTS_DIR_NAME="pc_scripts"
elif [[ $arch_name == aarch* ]]; then 
	echo "you're running on jetson!!!" 
	RUNNING_ON_JETSON=$true
	export RUNNING_ON_JETSON
	SCRIPTS_DIR_NAME="jetson_scripts"
fi

# export jetson_scripts or pc_scripts path to PATH variable
export PATH=$PATH:$ROS2_DOCKER_WS/$SCRIPTS_DIR_NAME/"build":$ROS2_DOCKER_WS/$SCRIPTS_DIR_NAME/"run":$ROS2_DOCKER_WS/$SCRIPTS_DIR_NAME/"utils"
export PATH=$PATH:$ROS2_DOCKER_WS"/system_scripts/edge":$ROS2_DOCKER_WS"/system_scripts/home" 

ROS_DEFAULT_DOMAIN_ID=12
export ROS_DEFAULT_DOMAIN_ID 

#create empty packages which git isn't monitoring, 
#and are needed for building the dockers and the packages correctly. 
source $ROS2_DOCKER_WS/system_scripts/setup/create_empty_folders.sh
