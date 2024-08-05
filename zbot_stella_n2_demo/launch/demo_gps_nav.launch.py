from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # gps_wpf_dir = get_package_share_directory(
    #     "nav2_gps_waypoint_follower_demo")
    
    # zbot_stella_n2_localization_dir = get_package_share_directory(
    #     "zbot_stella_n2_localization")
    
    # rl_params_file = os.path.join(
    #     zbot_stella_n2_localization_dir, "params", "dual_ekf_navsat_params.yaml")


    return LaunchDescription([

        # bring up robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_robot'), 'launch', 'bringup.launch.py']
            )),
            launch_arguments={
                'use_lidar': 'false',
                'use_rscam': 'false',
                'use_gps': 'true',
            }.items()
        ),

        # bring up localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_localization'), 'launch', 'dual_ekf_navsat.launch.py']
            )),
        )

    ])