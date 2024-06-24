from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    use_sim_time = launch.actions.LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        declare_use_sim_time_argument,
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory("zbot_stella_n2_localization"), 'params', 'ekf.yaml'),
                {"use_sim_time": use_sim_time},
            ],
           ),
    ])