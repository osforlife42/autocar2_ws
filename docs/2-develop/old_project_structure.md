# OLD project structure

```
.
├── build (the build directory for the host computer)
│   ├── async_mavros
│   ├── COLCON_IGNORE
│   ├── dorothy_robot
│   └── example_interfaces
├── docker_build (the build directory for the dorothy container)
│   ├── async_mavros
│   ├── basic_mobile_robot
│   ├── COLCON_IGNORE
│   ├── dorothy_robot
│   └── example_interfaces
├── docker_save (a shared volume to save files from the dorothy container in the computer)
│   ├── v1
│   ├── v2
│   └── v3
├── install (a shared volume for the docker - to easily run applications without rebuilding)
│   ├── async_mavros
│   ├── basic_mobile_robot
│   ├── COLCON_IGNORE
│   ├── dorothy_robot
│   ├── example_interfaces
│   ├── local_setup.bash
│   ├── local_setup.ps1
│   ├── local_setup.sh
│   ├── _local_setup_util_ps1.py
│   ├── _local_setup_util_sh.py
│   ├── local_setup.zsh
│   ├── setup.bash
│   ├── setup.ps1
│   ├── setup.sh
│   └── setup.zsh
├── know-hows.md
├── LICENSE
├── project_structure.md
├── README.md
├── ros1 (a folder for ros1 container - for ros1_bridge packages)
│   └── Dockerfile
├── scripts (scripts and utilities to setup, build and run the project)
│   ├── build
│   ├── run
│   ├── setup
│   └── utils
├── sitl (folder for a container containing - ardupilot, qgroundcontrol, mavproxy etc.. )
│   ├── configs
│   ├── Dockerfile
│   ├── docker_scripts
│   └── run_sitl_container.sh
├── sources.md
├── src (folder containing the source packages for the dorothy2_ws project )
│   ├── async_mavros
│   ├── basic_mobile_robot
│   ├── dorothy_robot
│   ├── example_interfaces
│   ├── mavros_bridge.repos
│   └── robot_localization
├── system_scripts (system scripts to run the whole system)
│   ├── edge
│   ├── home
│   └── tmux_test.sh
└── template_usage.md
```