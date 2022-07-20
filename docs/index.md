# dorothy2 workspace

## fetching and setup up
---

```
git clone https://192.168.1.6/zb/dorothy2_ws.git 
cd dorothy2_ws
git submodule update --init --recursive
source ./system_scripts/setup/setup_host_env.sh
```
setup_host_env.sh is a very important script to setup your project on each shell you are running,to read more about it [check this doc](1-run/setup_host_env.md)

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

there are many docker volume for your conveince and easy of use. to learn more about the dockers setup and methodology [check this doc](1-run/dockers_setup.md)

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

in the dorothy containers - dorothy2_ws folder is rested in the docker inside ~/dev_ws folder

docker_save is a volume to put files you want to save from the docker (before closing it)
there are more volumes for you for develop easyness.

for more details check [project structure](2-develop/project_structure.md)

## jetson installation
---
jetson installation is quite similar to pc installation, but with some important changes: 

1. setup [passwordless ssh](https://www.thegeekstuff.com/2008/11/3-steps-to-perform-ssh-login-without-password-using-ssh-keygen-ssh-copy-id/) - in order to the run scripts on the jetson you will need to set up ssh without password with. use this quick guide.

2. give your jetson a unique ip/hostname recoginzed by your pc to ssh easily. 

3. on the jetson - fetch and setup:
```
git clone https://192.168.1.6/zb/dorothy2_ws.git 
cd dorothy2_ws
git submodule update --init --recursive
source ./system_scripts/setup/setup_host_env.sh
```

4. building the dorothy docker on the jetson and first time package build: 
```
source ./system_scripts/setup/setup_host_env.sh
build_dorothy_docker.sh
build_dorothy_packages.sh
```

5. you're now ready to run a scenario with harsware in the loop. make sure `system_scripts/system_setup.json` has this configuration in it:
```json
{
    "components": {
        "machine": "jetson", 
        "jetson_machine_ip": "jetson_ip or hostname", (e.g. "192.168.1.69")
        "ardupilot": "sitl"
    },
    "domain_ids": {
        "edge_domain_id": 12,
        "home_domain_id": 11
    }
}
```
6. and run in two terminals on your pc!! (after sourcing setup_host_env.sh): 

```run_home.sh```

```run_sim.sh```

## template taken from
---

[how I develop with vscode and ros2](https://www.allisonthackston.com/articles/vscode_docker_ros2.html) for a more in-depth look on how to use this workspace.

[more information about this template](2-develop/template_usage.md)

## known problems and FAQ
---
[ROADMAP](2-develop/ROADMAP.md) for the project 

1. icons in gqroundcontrol aren't loaded and can't be seen.
2. the same in rviz


