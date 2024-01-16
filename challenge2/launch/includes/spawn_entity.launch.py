

# ros2 run gazebo_ros spawn_entity.py -entity test -database nuka_cola
# ros2 launch tbot_sim spawn_entity.launch.py entity:=nuka_cola

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    tbot_sim_path = get_package_share_directory('tbot_sim')
    gazebo_model_path = os.path.join(tbot_sim_path,'models')

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),

        DeclareLaunchArgument('entity', default_value='coke_can',
                              description='entity name to spawn (cf. https://github.com/osrf/gazebo_models)'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name="spawn_kobuki",
            output='screen',
            parameters=[],
            arguments=['-entity','kobuki', '-database', LaunchConfiguration('entity')])
        ])