#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node 

def generate_launch_description():
    tbot_sim_path = get_package_share_directory('tbot_sim')
    gazebo_model_path = os.path.join(tbot_sim_path,'models')
    worlds_model_path = os.path.join(tbot_sim_path,'worlds')

    world_arg = LaunchConfiguration('world')
    world_filepath = LaunchConfiguration('world_filepath')
    # world_file_path = DeclareLaunchArgument('world_filepath', default_value=[worlds_model_path, '/', world_arg, '.world'])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        # SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', worlds_model_path),
        
        DeclareLaunchArgument(
           'use_sim_time', 
           default_value='True',
           description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
           'world', 
           default_value='empty',
           description='World file to load'),
        
        DeclareLaunchArgument('world_filepath', default_value=[
            TextSubstitution(text=worlds_model_path),
            TextSubstitution(text='/'), 
            world_arg, 
            TextSubstitution(text='.world')]),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_filepath, 
                '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], 
                output='screen'),
        ])
