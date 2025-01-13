from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    
    config_file = PathJoinSubstitution([
        FindPackageShare('mowbot_imu_filter_madgwick'),
        'config',
        'imu_filter_madgwick.param.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[config_file],
        )
    ])