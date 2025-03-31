from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument('use_sim_time', 
        default_value='false', 
        description='Use simulation (Gazebo) clock if true'),
    
    DeclareLaunchArgument('dual_ekf_navsat_param_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('mowbot_robot_localization'),
            'config',
            'dual_ekf_navsat.param.yaml'
        ]),
        description='Path to the parameter file'),
]

def generate_launch_description():
    
    return LaunchDescription(ARGS + [
        Node(
            namespace='',
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[
                LaunchConfiguration('dual_ekf_navsat_param_path'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
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
            namespace='',
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[
                LaunchConfiguration('dual_ekf_navsat_param_path'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
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
            namespace='',
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                LaunchConfiguration('dual_ekf_navsat_param_path'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                #input
                ("odometry/filtered", "odometry/global"),
                ("gps/fix", "/gnss_fused/fix"),
                ("imu", "imu/data"),
                #output
                ("odometry/gps", "odometry/gnss"),
                ("gps/filtered", "gnss/filtered"),
            ]
        ),
    ])