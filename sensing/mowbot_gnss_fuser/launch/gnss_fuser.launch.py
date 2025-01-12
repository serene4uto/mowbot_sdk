from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGS = []

def generate_launch_description():
    
    gnss_fuser_config = PathJoinSubstitution([
        FindPackageShare('mowbot_gnss_fuser'),
        'config',
        'gnss_fuser.param.yaml'
    ])
    
    return LaunchDescription(ARGS + [
        Node(
            package='mowbot_gnss_fuser',
            executable='gnss_fuser',
            name='gnss_fuser',
            output='screen',
            parameters=[gnss_fuser_config],
            remappings=[
                ('/left/fix', '/gnss_left/fix'),
                ('/right/fix', '/gnss_right/fix')
            ],
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
    ])