from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os





def generate_launch_description():
    imu_filter_params = os.path.join(get_package_share_directory('zbot_stella_n2_gazebo'),
                                        'params', 'gazebo_imu_filter.yaml')
        
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

    # imu filter
    imu_madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
        output='screen',
        parameters=[
            imu_filter_params,
            {'use_sim_time': 'true'},
        ],
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory("zbot_stella_n2_gazebo"), 'params', 'gazebo_ekf.yaml'),
            {"use_sim_time": True},
        ],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ]
    )

    # load control
    spawn_control_timer_action = TimerAction(
        period=5.0,
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
    ld.add_action(imu_madgwick_filter_node)
    ld.add_action(robot_localization_node)
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
