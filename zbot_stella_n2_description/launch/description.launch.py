from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('zbot_stella_n2_description'), 'urdf', 'zbot_stella_n2.urdf.xacro']
            ),
            ' ',
            'is_sim:=', use_sim_time
        ])
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',    
        namespace='',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
    )


    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(robot_state_publisher_node)
    

    return ld