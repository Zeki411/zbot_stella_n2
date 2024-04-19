from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    namespace_declare = DeclareLaunchArgument('namespace', default_value='', description='Namespace of the robot')
    use_sim_time_declare = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation/Gazebo clock')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_description'), 'urdf', 'main.urdf.xacro']
            )
        ])
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',    
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }],
    )


    ld = LaunchDescription()
    ld.add_action(namespace_declare)
    ld.add_action(use_sim_time_declare)

    ld.add_action(robot_state_publisher_node)

    return ld