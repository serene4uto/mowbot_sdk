#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the package name for use with paths
    package_name = 'mowbot_description'

    # Set up the path to the URDF file using xacro
    urdf_file = PathJoinSubstitution(
        [FindPackageShare(package_name), 'urdf', 'mowbot.urdf.xacro']
    )

    # Declare the `use_sim_time` launch argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace of the robot'
    )
    namespace = LaunchConfiguration('namespace')
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Log info for parameter loading
    log_info = LogInfo(msg=['Loading robot description from: ', urdf_file])

    # Define robot description content using xacro command
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            urdf_file, ' ',
            'is_sim:=', use_sim_time
        ]),
        value_type=str
    )

    # Define the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        log_info,
        robot_state_publisher_node
    ])
