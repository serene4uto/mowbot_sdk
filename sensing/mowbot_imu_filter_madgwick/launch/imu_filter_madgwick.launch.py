from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('imu_filter_madgwick_param_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('mowbot_imu_filter_madgwick'),
            'config',
            'imu_filter_madgwick.param.yaml'
        ]),
        description='Path to the parameter file'),
]

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick_node',
            output='screen',
            parameters=[LaunchConfiguration('imu_filter_madgwick_param_path')],
        )
    ])