from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.actions
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    nav2_file_dir = get_package_share_directory('zbot_stella_n2_navigation')
    param_file_name = 'nav2_2d_no_map_params.yaml'

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')


    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_file_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            # 'map': os.path.join(nav2_file_dir, 'map', 'map.yaml'),
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(nav2_file_dir, 'params', param_file_name)}.items(),
    )

    return LaunchDescription([
        declare_use_sim_time_argument,
        nav2_bringup_launch,
    ])