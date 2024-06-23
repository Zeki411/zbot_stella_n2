from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rviz_config_path = PathJoinSubstitution([FindPackageShare('zbot_stella_n2_description'), 'rviz', 'rviz_config.rviz'])

    namespace_declare = DeclareLaunchArgument('namespace', default_value='', description='Namespace of the robot')
    use_sim_time_declare = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation/Gazebo clock')
    use_gui_declare = DeclareLaunchArgument('use_gui', default_value='true', description='Use GUI for joint state publisher')
    rviz_config_declare = DeclareLaunchArgument('rviz_config_path', default_value=rviz_config_path, description='RViz config file')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('zbot_stella_n2_description'), 
                'launch', 
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=namespace,
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_path')],
    )


    ld = LaunchDescription()
    ld.add_action(namespace_declare)
    ld.add_action(use_sim_time_declare)
    ld.add_action(use_gui_declare)
    ld.add_action(rviz_config_declare)

    ld.add_action(robot_description_launch)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)


    return ld