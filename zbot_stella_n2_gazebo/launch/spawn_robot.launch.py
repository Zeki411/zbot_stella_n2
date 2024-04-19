from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit



def generate_launch_description():
    
    # Declare launch arguments for robot name and initial XYZ position
    declare_namespace = DeclareLaunchArgument('namespace', default_value='', description='Namespace of the robot')
    declare_x = DeclareLaunchArgument('x', default_value='0', description='Initial x position of the robot')
    declare_y = DeclareLaunchArgument('y', default_value='0', description='Initial y position of the robot')
    declare_z = DeclareLaunchArgument('z', default_value='0', description='Initial z position of the robot')

    # Use LaunchConfigurations for dynamic parameter retrieval
    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')


    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zbot_stella_n2_description'), 
                'launch', 
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': 'true',
        }.items()
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace,
        arguments=['-entity', namespace, 
                   '-topic', 'robot_description', 
                   '-x', x, '-y', y, '-z', z, '-timeout', '30']
    )


    # Assemble the Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)

    ld.add_action(robot_description_launch)
    ld.add_action(spawn_robot_node)

    # ld.add_action(
    #     RegisterEventHandler(
    #         OnProcessExit(
    #             target_action=robot_description_launch,
    #             on_exit=[spawn_robot_node],
    #         )
    #     )
    # )

    return ld
