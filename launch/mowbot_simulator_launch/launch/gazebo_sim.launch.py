from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


from ament_index_python.packages import get_package_share_directory

from pathlib import Path

ARGUMENTS = [
    DeclareLaunchArgument('headless', default_value='false',
                            description='Whether to launch gazebo in headless mode'),
    DeclareLaunchArgument('world_path', default_value='',
                            description='The world path, by default is empty.world'),
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

    # this is the path to the gazebo models
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
                                                EnvironmentVariable('GAZEBO_MODEL_PATH',
                                                                    default_value=''),
                                                '/usr/share/gazebo-11/models/:',
                                                str(Path(get_package_share_directory('mowbot_gazebo_modelv3_description')).
                                                    parent.resolve())])
    # Launch args
    world_path = LaunchConfiguration('world_path')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )
    
    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless'))
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
    
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mowbot_robot_launch'),
                'launch',
                'robot.launch.py'
            ])
        ),
        launch_arguments={
            'launch_robot_interface': 'true',
            'model': 'mowbot_gazebo_modelv3',
            'sensor_model': 'mowbot_gazebo_sensor_kit',
            'config_dir': PathJoinSubstitution(
                [FindPackageShare('mowbot_gazebo_sensor_kit_description'), 'config']
            )
        }.items()
    )
    
    

    return LaunchDescription(ARGUMENTS + [
        gz_resource_path,
        gzserver,
        gzclient,
        robot_launch,
        spawn_robot_node,
    ])