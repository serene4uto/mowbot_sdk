from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


ARGS = [
    # DeclareLaunchArgument('dual_ekf_navsat_param_path',
    #     default_value='',
    #     description='Path to the parameter file'),
]

def generate_launch_description():
    
    return LaunchDescription(ARGS + [
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('mowbot_robot_localization'),
                    'launch',
                    'dual_ekf_navsat.launch.py'
                ])
            ),
            launch_arguments={
            }.items()
        ),
        
    ])