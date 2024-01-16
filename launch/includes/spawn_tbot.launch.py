import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    tbot_sim_path = get_package_share_directory('tbot_sim')
    gazebo_model_path = os.path.join(tbot_sim_path,'models')
    kobuki_urdf = os.path.join(tbot_sim_path,'urdf', 'tbot.urdf')
    urdf = open(kobuki_urdf).read()

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        DeclareLaunchArgument(
           'use_sim_time',
           default_value='true',
           description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name="robot_state_publisher",
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                'robot_description': urdf}]),

        # TODO: spawn tbot from URDF into gazebo instead of SDF
        # https://automaticaddison.com/how-to-load-a-urdf-file-into-gazebo-ros-2/
        # Node(
        #     package='gazebo_ros', 
        #     executable='spawn_entity.py',
        #     arguments=['-entity', 'tbot', 
        #                 '-topic', 'robot_description'],
        #     output='screen')
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name="spawn_kobuki",
            output='screen',
            parameters=[],
            arguments=['-entity','tbot', '-database', 'tbot']),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name="rviz2",
        #     output='screen'),

])