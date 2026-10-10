from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    map_topic = LaunchConfiguration("map_topic")

    cloud_map_launch = PathJoinSubstitution([
        FindPackageShare("go2_cloud_map_projector"),
        "launch",
        "go2_cloud_map.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_deskewed"),
        DeclareLaunchArgument("map_topic", default_value="/map_from_cloud"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cloud_map_launch),
            launch_arguments={
                "cloud_topic": cloud_topic,
                "map_topic": map_topic,
            }.items(),
        ),
    ])
