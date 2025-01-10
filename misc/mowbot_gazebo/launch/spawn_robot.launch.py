from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGS = [
    DeclareLaunchArgument('robot_name', default_value='mowbot',
                            description='The name of the robot to spawn'),
    DeclareLaunchArgument('x', default_value='0.0',
                            description='The x position of the robot to spawn'),
    DeclareLaunchArgument('y', default_value='0.0',
                            description='The y position of the robot to spawn'),
    DeclareLaunchArgument('z', default_value='0.0',
                            description='The z position of the robot to spawn'),
    DeclareLaunchArgument('yaw', default_value='0.0',
                            description='The yaw position of the robot to spawn'),
]


def generate_launch_description():
    
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    
    config_mowbot_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("mowbot_gazebo"), "config", "gazebo_control.yaml"]
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mowbot_description"), "urdf", "mowbot.urdf.xacro"]
            ),
            " ",
            "name:=",
            robot_name,
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_mowbot_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )
    
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )
    
    spawn_robot_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mowbot_velocity_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_mowbot_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_robot_velocity_controller],
        )
    )
    
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace='',
        arguments=['-entity', robot_name,
                   '-topic', 'robot_description', 
                   '-x', x, '-y', y, '-z', z, '-Y', yaw,
                   '-timeout', '30']
    )
    
    bringup_group = GroupAction([
        # twist_mux
        Node(
            name='twist_mux',
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={
                ('/cmd_vel_out', '/mowbot_velocity_controller/cmd_vel_unstamped')
            },
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_bringup'), 'config', 'twist_mux.yaml']
                )
            ]
        ),
        
        # teleop
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            name='joy_node',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_gazebo'), 'config', 'teleop.yaml']
                )
            ]
        ),
        
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('mowbot_gazebo'), 'config', 'teleop.yaml']
                )
            ],
            remappings=[
                ('/cmd_vel', '/joy_cmd_vel')
            ]
        ),
        
    ])
    
    

    return LaunchDescription(ARGS + [
        node_robot_state_publisher,
        spawn_joint_state_broadcaster,
        diffdrive_controller_spawn_callback,
        spawn_robot_node,
        bringup_group,
    ])