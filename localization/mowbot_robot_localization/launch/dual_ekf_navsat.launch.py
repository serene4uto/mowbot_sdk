from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = []

def generate_launch_description():
    
    config_path = PathJoinSubstitution([
        FindPackageShare('mowbot_robot_localization'),
        'config',
        'dual_ekf_navsat.param.yaml'
    ])
    
    return LaunchDescription(ARGS + [
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[
                config_path,
            ],
            remappings=[
                #input
                ("odom", "mowbot_base/odom"),
                ("imu", "imu/data"),
                #output
                ("odometry/filtered", "odometry/local")
            ]
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[
                config_path,
            ],
            remappings=[
                #input
                ("odom", "mowbot_base/odom"),
                ("imu", "imu/data"),
                ("odometry/gps", "odometry/gnss"),
                #output
                ("odometry/filtered", "odometry/global")
            ]
        ),

        Node(
            namespace=LaunchConfiguration('namespace'),
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                config_path,
            ],
            remappings=[
                #input
                ("odometry/filtered", "odometry/global"),
                ("gps/fix", "/gnss/fix"),
                ("imu", "imu/data"),
                #output
                ("odometry/gps", "odometry/gnss"),
                ("gps/filtered", "gnss/filtered"),
            ]
        ),
            
    ])