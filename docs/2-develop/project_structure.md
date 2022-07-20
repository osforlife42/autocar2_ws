# NEW project structure

```
.
├── build (the build directory for the host computer)
│   ├── async_mavros
│   ├── basic_mobile_robot
│   ├── COLCON_IGNORE
│   ├── cv_utils
│   ├── domain_bridge
│   ├── dorothy_robot
│   ├── example_interfaces
│   └── robot_localization
├── docker_build (the build directory for the dorothy container)
│   ├── async_mavros
│   ├── basic_mobile_robot
│   ├── COLCON_IGNORE
│   ├── costmap_converter
│   ├── costmap_converter_msgs
│   ├── cv_utils
│   ├── domain_bridge
│   ├── dorothy_robot
│   ├── example_interfaces
│   ├── robot_localization
│   ├── teb_local_planner
│   └── teb_msgs
├── docker_save (a shared volume to save files from the dorothy container in the computer)
│   ├── autocar
│   ├── v1
│   ├── v2
│   ├── ...
│   ├── v8
│   └── v9
├── docs
│   ├── 1-run
│   ├── 2-develop
│   ├── 3-learn
│   ├── index.md
│   ├── mermaid.min.js
│   ├── new_project_structure.md
│   └── sources.md
├── edge (a folder for the jetson edge container - containing how to build the jetson packages)
│   ├── Dockerfile
│   ├── galactic_jetson_base
│   ├── jetson_base
│   └── scripts
├── install (a shared volume for the dorothy docker - to easily run applications without rebuilding)
│   ├── async_mavros
│   ├── basic_mobile_robot
│   ├── COLCON_IGNORE
│   ├── costmap_converter
│   ├── costmap_converter_msgs
│   ├── cv_utils
│   ├── domain_bridge
│   ├── dorothy_robot
│   ├── example_interfaces
│   ├── local_setup.bash
│   ├── local_setup.ps1
│   ├── local_setup.sh
│   ├── _local_setup_util_ps1.py
│   ├── _local_setup_util_sh.py
│   ├── local_setup.zsh
│   ├── robot_localization
│   ├── setup.bash
│   ├── setup.ps1
│   ├── setup.sh
│   ├── setup.zsh
│   ├── teb_local_planner
│   └── teb_msgs
├── jetson_scripts  (jetson scripts and utilities to setup, build and run the project)
│   ├── build
│   ├── run
│   └── utils
├── LICENSE
├── log (log folder for the host build process )
│   ├── ...
│   ├── build_2022-07-11_09-25-20
│   ├── COLCON_IGNORE
│   ├── latest -> latest_build
│   └── latest_build -> build_2022-07-11_09-25-20
├── mkdocs.yml
├── pc_scripts (pc scripts and utilities to setup, build and run the project)
│   ├── build
│   ├── run
│   └── utils
├── README.md
├── ros1 (a folder for ros1 container - for ros1_bridge packages)
│   └── Dockerfile
├── simulation (folder for a container containing simulation packages - gazebo, gazeob_ros_pkgs,  etc.) 
│   ├── Dockerfile
│   └── scripts
├── sitl (folder for a container containing - ardupilot, qgroundcontrol, mavproxy etc.. )
│   ├── configs
│   ├── Dockerfile
│   ├── docker_scripts
│   └── run_sitl_container.sh
├── src (folder containing the source packages for the dorothy2_ws project )
│   ├── async_mavros
│   ├── basic_mobile_robot
│   ├── costmap_converter
│   ├── cv_utils
│   ├── domain_bridge
│   ├── dorothy_robot
│   ├── example_interfaces
│   ├── mavros_bridge.repos
│   ├── robot_localization
│   └── teb_local_planner
├── system_scripts (system scripts to run and setup the whole system)
│   ├── edge
│   ├── home
│   ├── setup
│   ├── system_setup.json
│   └── tmux_test.sh
└── ui (folder for a container containing - ui and home applications: rviz, qgroundcontrol, joystick node, domain_bridge etc. )
    ├── Dockerfile
    └── scripts

```
