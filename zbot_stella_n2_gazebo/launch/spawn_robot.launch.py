from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit





def generate_launch_description():
    declare_robot_name = DeclareLaunchArgument('robot_name', default_value='stella_n2', description='Name of the robot to be spawned')
    
    # Declare launch arguments for robot name and initial XYZ position
    declare_x = DeclareLaunchArgument('x', default_value='0', description='Initial x position of the robot')
    declare_y = DeclareLaunchArgument('y', default_value='0', description='Initial y position of the robot')
    declare_z = DeclareLaunchArgument('z', default_value='0', description='Initial z position of the robot')

    robot_name = LaunchConfiguration('robot_name')

    # Use LaunchConfigurations for dynamic parameter retrieval
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
            'use_sim_time': 'true'
        }.items()
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='',
        arguments=['-entity', robot_name,
                   '-topic', 'robot_description', 
                   '-x', x, '-y', y, '-z', z, '-timeout', '30']
    )

    #teleop
    teleop_group_action = GroupAction([

        # Base Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_control'), 'launch', 'teleop_base.launch.py']
            )),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Joy Teleop
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_control'), 'launch', 'teleop_joy.launch.py']
            )),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

    ])
    

    # load control
    spawn_control_timer_action = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen',
            ),

            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['zbot_stella_n2_velocity_controller'],
                output='screen',
            ),
        ]
    )




    # Assemble the Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_robot_name)
    ld.add_action(declare_x)
    ld.add_action(declare_y)
    ld.add_action(declare_z)

    ld.add_action(robot_description_launch)
    ld.add_action(spawn_robot_node)
    ld.add_action(teleop_group_action)
    ld.add_action(spawn_control_timer_action)

    # ld.add_action(
    #     RegisterEventHandler(
    #         OnProcessExit(
    #             target_action=robot_description_launch,
    #             on_exit=[spawn_robot_node],
    #         )
    #     )
    # )

    return ld
