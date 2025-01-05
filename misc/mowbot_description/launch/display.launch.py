#!/usr/bin/env python3
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Declare the `use_sim_time` launch argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace of the robot'
    )
    namespace = LaunchConfiguration('namespace')
    
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Launch joint_state_publisher_gui'
    )
    use_gui = LaunchConfiguration('use_gui')
        
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('mowbot_description'),
        'rviz',
        'mowbot.rviz'
    ])
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=rviz_config_path,
        description='Path to the RViz config file to use'
    )
    
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mowbot_description'), 
                'launch', 
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        condition=UnlessCondition(use_gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        namespace=namespace,
        condition=IfCondition(use_gui)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_path')],
    )

    return LaunchDescription([
        namespace_arg,
        use_gui_arg,
        use_sim_time_arg,
        rviz_config_arg,
        robot_description_launch,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
