# dorothy2 workspace

## about 
this project is a based largely on a foundation from: 

1. [ultimate guide to the ros2 navigation](https://automaticaddison.com/the-ultimate-guide-to-the-ros-2-navigation-stack/) - it is highly recommended to go through before digging in to this project. 

2.  [project template](https://github.com/athackst/vscode_ros2_workspace) - just to understand the structure of the workspace, working with vscode and dockers.

## fetching and setup up
---

```
git clone https://192.168.1.6/zb/dorothy2_ws.git 
cd dorothy2_ws
git submodule update --init --recursive
source ./system_scripts/setup/setup_host_env.sh
```
setup_host_env.sh is a very important script to setup your project on each shell you are running.

## getting started 
---
### pc installation
```
source ./system_scripts/setup/setup_host_env.sh
build_dorothy_docker.sh
build_sim_docker.sh
build_ui_docker.sh
build_dorothy_packages.sh
```

`build_dorothy_docker.sh`, `build_sim_docker.sh`, `build_ui_docker.sh` scripts builds the relevant dockers on your pc.

`build_dorothy_packages.sh` script builds the relevant packages on the edge docker.  


### running

you're now ready to run a the project on your pc. 

**again - in order for any of the scripts to work you have to source setup_host_env.sh in your current shell** e.g.:
```
cd dorothy2_ws
source ./scripts/setup/setup_host_env.sh
```

the system scripts are the one you are looking for running the whole system. they use `tmux` so make sure you have it on the host system:

```sudo apt install tmux```

also, in order to parse system configuration files written in json, install `jq`:

```sudo apt install jq```

a scenario run setup is controlled by `system_scripts/system_setup.json` file. to start, make sure it has this configuration in it: 
```json
{
    "components": {
        "machine": "pc", 
        "jetson_machine_ip": "192.168.1.69", 
        "ardupilot": "sitl"
    },
    "domain_ids": {
        "edge_domain_id": 12,
        "home_domain_id": 11
    }
}
```

there is a distinction between home and edge scripts:

```run_home.sh``` - runs the user interface: qgc, rviz, rqt, etc...

```run_sim.sh``` - runs the edge componenets in the simulation: gazebo, sitl, ros2 edge nodes
(later there will be run_edge_robot.sh script which will run on the real-world edge)

after changing the above json file, open two terminals and in both of them ```source ./system_scripts/setup/setup_host_env.sh``` and launch:

in terminal 1 - ```run_home.sh```

in terminal 2 - ```run_sim.sh```

these scripts open a tmux terminal and in each window open a container and start relevant programs.
the scripts which run each docker are rested inside pc_scripts/run/ folder.

for more details about running and understanding the project structure look through mkdocs. 

the project is also meant to be run with hardware in the loop, to use and develop check the mkdocs. 

## more documentation and mkdocs pages
---

in order to build a correct mkdocs web pages install the following:

```pip install markdown-include mkdocs-mermaid2-plugin mkdocstrings mkdocs-material-extensions```

and then in the root project directory you run the page with:

```mkdocs serve```

## known problems and FAQ
---
roadmap for the project can be found in the mkdocs documentation.

1. icons in gqroundcontrol aren't loaded and can't be seen.
2. the same in rviz


