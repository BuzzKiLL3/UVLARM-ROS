#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    tbot_sim_path = get_package_share_directory('tbot_sim')
    gazebo_model_path = os.path.join(tbot_sim_path,'models')
    launch_file_dir = os.path.join(tbot_sim_path, 'launch','includes')
    worlds_model_path = os.path.join(tbot_sim_path,'worlds')

    world_arg = LaunchConfiguration('world')
    world_filepath = LaunchConfiguration('world_filepath')

    return LaunchDescription([
        DeclareLaunchArgument(
           'world',
           default_value='challenge-1',
           description='World file to load'),

        DeclareLaunchArgument('world_filepath', default_value=[
            TextSubstitution(text=worlds_model_path),
            TextSubstitution(text='/'),
            world_arg,
            TextSubstitution(text='.world')]),

        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/gazebo.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'world' : world_filepath}.items(),
        ),

        # Spawn tbot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/spawn_tbot.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Spawn kobuki
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/spawn_kobuki.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),

        # Spawn create
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/spawn_create.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),
    ])
