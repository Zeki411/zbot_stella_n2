# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, GroupAction
# from launch.conditions import UnlessCondition
# from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch_ros.parameter_descriptions import ParameterValue


# def generate_launch_description():



#     # ROS2 Controls
#     control_group_action = GroupAction([


#         # Velocity Controller
#         Node(
#             package='controller_manager',
#             executable='spawner',
#             arguments=['jackal_velocity_controller'],
#             output='screen',
#         )
#     ])

#     # Launch Description
#     ld = LaunchDescription()
#     ld.add_action(robot_description_command_arg)
#     ld.add_action(is_sim_arg)
#     ld.add_action(localization_group_action)
#     ld.add_action(control_group_action)

#     return ld