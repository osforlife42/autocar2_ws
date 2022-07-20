# know-hows 

### start the turtlebot slam simulation
```
ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
```

### ros2 echo some of the tf2 messages:
```
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo <parent frame> <child frame>
```
### launchConfiguration vs. DeclareLaunchArgument
1. parameters are controlled with LaunchConfiguration instances
2. LaunchConfiguration can be controlled with other launch-files, command-line, or configuration-file using DeclareLaunchArgument. [example of the difference](https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/)
3. [launch architechture](https://github.com/ros2/launch/blob/galactic/launch/doc/source/architecture.rst)

### urdf vs. sdf
1. [automatic addison to the rescue](https://automaticaddison.com/urdf-vs-sdf-link-pose-joint-pose-visual-collision/)

### robot localization package 
remember to install to package: ```sudo apt install ros-${ROS_DISTRO}-robot-localization``` 
and also to clone parameters fix from the source and build it again e.g.:
```
cd /path/to/workspace/
cd src 
git clone -b fix/galactic/load_parameters https://github.com/nobleo/robot_localization.git
cd .. 
colcon build
```

### localization with a known map (IMPORTANT): 
1. [navigation2 getting started](https://navigation.ros.org/getting_started/index.html#getting-started) 
After starting, the robot initially has no idea where it is. By default, Nav2 waits for you to give it an approximate starting position. Take a look at where the robot is in the Gazebo world, and find that spot on the map. Set the initial pose by clicking the “2D Pose Estimate” button in RViz, and then down clicking on the map in that location. You set the orientation by dragging forward from the down click.

If you are using the defaults so far, the robot should look roughly like this.

The key to getting good performance with the ROS 2 Navigation Stack is to spend a lot of time (it can take me several days) tweaking the parameters in the nav2_params.yaml file we built earlier. Yes, it is super frustrating, but this is the only way to get navigation to work properly. 

Common things you can try changing are the robot_radius and the inflaition_radius parameters. You can also try changing the expected_planner_frequency, update_frequency, publish_frequency, and width/height of the rolling window in the local_costmap.

Also, you can try modifying the update_rate in the LIDAR sensor inside your robot model.sdf file.

### saving a map 
```ros2 run nav2_map_server map_saver_cli -f /full/path/to/saved_map``` 