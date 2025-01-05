from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


ARGS = [
]


def generate_launch_description():
    
    world_path = PathJoinSubstitution([FindPackageShare('mowbot_gazebo'),'worlds','sonoma_raceway.world'])
    return LaunchDescription(ARGS + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mowbot_gazebo'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ),
            launch_arguments={
                'world_path': world_path,
            }.items()
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mowbot_gazebo'),
                    'launch',
                    'spawn_robot.launch.py'
                ])
            ),
            launch_arguments={
                'robot_name': 'mowbot',
                'x': '0',
                'y': '0',
                'z': '2',
                'yaw': '0',
            }.items()
        ),
     
    ])