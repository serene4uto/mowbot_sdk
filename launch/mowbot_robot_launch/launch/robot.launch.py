from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    SetLaunchConfiguration, 
    GroupAction, 
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node




ARGS = [
    DeclareLaunchArgument(
        'launch_robot_interface',
        default_value='false',
        description='Launch the robot interface'
    ),
    DeclareLaunchArgument(
        'model',
        default_value='mowbot_gazebo_modelv3',
        description='mowbot model name'
    ),
    DeclareLaunchArgument(
        'sensor_model',
        default_value='mowbot_default_sensor_kit',
        description='sensor model name'
    ),
    DeclareLaunchArgument(
        'config_dir',
        default_value=[
            LaunchConfiguration('sensor_model'),
            '_description',
            '/config'
        ],
        description='Path to the directory where sensors_calibration.yaml, etc. are located'
    ),
    
    DeclareLaunchArgument(
        'model_file',
        default_value=[
            FindPackageShare('mowbot_robot_launch'),
            '/urdf/robot.xacro'
        ],
        description="path to the file of model settings (*.xacro)"
    ),
]

def generate_launch_description():

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
            LaunchConfiguration('model_file'), ' ',
            'robot_model:=', LaunchConfiguration('model'), ' ',
            'sensor_model:=', LaunchConfiguration('sensor_model'), ' ',
            'config_dir:=', LaunchConfiguration('config_dir'), ' ',
        ]),
        value_type=str
    )
    
    return LaunchDescription(ARGS + [
        
        SetLaunchConfiguration(
            name='mowbot_launch_pkg',
            value=[
                LaunchConfiguration('model'),
                '_launch'
            ]
        ),
        
        GroupAction([
            Node(
                namespace='',
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{
                    'robot_description': robot_description_content
                }]
            )
        ]),
        
        #TODO: robot interface
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare(LaunchConfiguration('mowbot_launch_pkg')),
                    'launch',
                    'robot_interface.launch.py'
                ])
            ),
            condition=IfCondition(LaunchConfiguration('launch_robot_interface'))
        )
        
    ])