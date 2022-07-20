'''
a launch file containing only the ui components for the dorothy robot package
'''
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_share = FindPackageShare(package='dorothy_robot').find('dorothy_robot')
    world_file_name = 'turtlebot3_house.world'
    # world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    default_model_path = os.path.join(
        pkg_share, 'models/dorothy_ackermann.xacro')
    # static_map_path = os.path.join(
    #     pkg_share, 'maps', 'turtlebot3_house_map.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/teb_and_slam_config.rviz')

    # launch configurations
    # model = LaunchConfiguration('model')
    # namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # common remappings
    # remappings = [('/tf', 'tf'),
    #               ('/tf_static', 'tf_static')]

    # launch configurations declarations
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_model_path_cmd = DeclareLaunchArgument(
        name='model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])

    start_joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_share, 'launch', 'teleop_joy.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,}.items())
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_joy_node)

    return ld
