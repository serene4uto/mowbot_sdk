from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGS = [
    DeclareLaunchArgument(
        "fix_topic",
        default_value="fix",
        description="GPS fix topic"
    ),

    DeclareLaunchArgument(
        "mvc_config",
        default_value="",
        description="Mapviz config file"
    ),
]

def generate_launch_description():

    return LaunchDescription(ARGS + [

        Node(
            namespace='',
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            parameters=[{"config": LaunchConfiguration("mvc_config")}],
        ),

        Node(
            namespace='',
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            remappings=[
                ("fix", LaunchConfiguration("fix_topic")),
            ],
        ),

        Node(
            namespace='',
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        ),

    ])