# Author: Addison Sears-Collins
# Date: August 27, 2021
# Description: Launch a basic mobile robot URDF file using Rviz.
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.
  
  # Specify the actions

  # Publish the joint state values for the non-fixed joints in the URDF file.
  async_mavros_node = Node(package='async_mavros', executable='async_mavros', name='async_mavros', remappings=[], )
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(async_mavros_node)

  return ld