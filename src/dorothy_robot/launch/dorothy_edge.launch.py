
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    # pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package='dorothy_robot').find('dorothy_robot')
    default_model_path = os.path.join(
        pkg_share, 'models/dorothy_ackermann.xacro')
    static_map_path = os.path.join(
        pkg_share, 'maps', 'turtlebot3_house_map.yaml')

    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml')

    # launch configurations
    namespace = LaunchConfiguration('namespace')
    slam = LaunchConfiguration('slam')
    params_file = LaunchConfiguration('params_file', default=os.path.join(
        pkg_share, 'params', 'nav2_params.yaml'))

    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    against_simulator = LaunchConfiguration('against_simulator')

    # common remappings
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # launch configurations decalarations
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        name='slam',
        default_value='False',
        description='Whether to run SLAM')

    declare_against_simulator = DeclareLaunchArgument(
        name='against_simulator',
        default_value='True',
        description='Whether this nodes run against simulation or against the real robot')

    # Specify the actions
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': "false",
                          'slam': slam,
                          'use_sim_time': use_sim_time,
                          'map': static_map_path,
                          'params_file': params_file,
                          'autostart': "true"}.items())

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path,
                    {'use_sim_time': use_sim_time}])

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': Command(['xacro ', model])}],
        remappings=remappings,
        arguments=[default_model_path])

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_against_simulator)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_slam_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_ros2_navigation_cmd)

    return ld 