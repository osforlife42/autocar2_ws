# containers setup 

**dorothy2-ui:dev**: the ui docker - containig rviz and other ui applications at home side 

**dorothy2-sim:dev**: containig gazebo and it's ros plugins which enable to run the simulation 

**dorothy2-pc:dev**: the main docker for running dorothy2 nodes on the pc

**dorothy2-edge:dev**: the main docker for running dorothy2 nodes on the jetson


keep in mind: `~/dev_ws` is a volume to your workspace project (the path is saved as ROS2_DOCKER_WS environment variable )

for each of the dockers there should be the following scripts: 

1. a building script, e.g. `build_ui_docker.sh` `build_dorothy_docker.sh`, which builds the relevant docker with all it's dependencies. 

2. a running script, e.g. `run_ui_docker.sh` `run_dorothy_docker.sh`, which opens an interactive terminal inside this docker with all of the relevant volumes configured. 

3. a starting script, e.g. `start_dorothy_ui.sh` `start_dorothy_edge.sh`, which runs the docker and starts it's main entrypoint script / ros2 launch file. 

4. a build packages script, e.g. `build_dorothy_packages.sh`, which builds all the packages with  --symlink-install --packages-select the relevant packages to the docker.

all the scripts should be found inside `jetson_scripts` folder or `pc_scripts` folder, and are added to your path automatically when you [`source setup_host_env.sh`](../1-run/setup_host_env.md)