from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

ARGS = [
    DeclareLaunchArgument(
        'model',
        default_value='mowbot',
        description='mowbot model name'
    ),
]

def generate_launch_description():
    # Define the mowbot_launch_pkg variable
    mowbot_launch_pkg = PathJoinSubstitution([
        FindPackageShare(LaunchConfiguration('model')),
        LaunchConfiguration('model'),
        TextSubstitution(text='_launch')
    ])

    return LaunchDescription(ARGS + [
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    
                ])
            ),
            launch_arguments={
            }.items()
        )
            
    ])