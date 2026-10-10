from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration("params")
    cloud_topic = LaunchConfiguration("cloud_topic")
    map_topic = LaunchConfiguration("map_topic")

    default_params = PathJoinSubstitution([
        FindPackageShare("go2_cloud_map_projector"),
        "config",
        "cloud_map_projector.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_deskewed"),
        DeclareLaunchArgument("map_topic", default_value="/map_from_cloud"),

        Node(
            package="go2_cloud_map_projector",
            executable="go2_cloud_map_projector",
            name="go2_cloud_map_projector",
            output="screen",
            parameters=[
                params,
                {
                    "cloud_topic": cloud_topic,
                    "map_topic": map_topic,
                },
            ],
        ),
    ])
