
'''
only launch simulation of dorothy robot
'''

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package='dorothy_robot').find('dorothy_robot')

    world_file_name = 'turtlebot3_house.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    default_model_path = os.path.join(
        pkg_share, 'models/dorothy_ackermann.xacro')

    #launch configurations 
    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless', default="False")

    # common remappings  
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # launch configuration declarations
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    # Specify the actions
    # gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items())

    # gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=UnlessCondition(headless))
    
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    return ld 