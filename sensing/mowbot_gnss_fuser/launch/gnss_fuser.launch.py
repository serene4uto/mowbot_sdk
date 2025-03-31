from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('gnss_fuser_param_path', 
        default_value=PathJoinSubstitution([
            FindPackageShare('mowbot_gnss_fuser'),
            'config',
            'gnss_fuser.param.yaml'
        ]), 
        description='Path to the parameter file'),
]

def generate_launch_description():
    
    return LaunchDescription(ARGS + [
        Node(
            package='mowbot_gnss_fuser',
            executable='gnss_fuser',
            name='gnss_fuser',
            output='screen',
            parameters=[LaunchConfiguration('gnss_fuser_param_path')],
            remappings=[
                ('/left/fix', '/gnss_left/fix'),
                ('/right/fix', '/gnss_right/fix')
            ],
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
    ])