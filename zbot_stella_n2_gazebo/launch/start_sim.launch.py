from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_world_config = PathJoinSubstitution([FindPackageShare('zbot_stella_n2_gazebo'), 'worlds', 'no_roof_small_warehouse.world'])
   
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 
                'launch', 
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': gazebo_world_config,
            'verbose': 'true',
            'paused': 'true',
            'gui': 'true',
            # 'physics': 'ode',
        }.items()
    )


    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zbot_stella_n2_gazebo'), 
                'launch', 
                'spawn_robot.launch.py'
            ])
        ),
        launch_arguments={
            'x': '0',
            'y': '0',
            'z': '0.1',
        }.items()
    )

    


    ld = LaunchDescription()

    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robot_launch)


    return ld
