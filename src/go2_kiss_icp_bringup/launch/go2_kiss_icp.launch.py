from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    kiss_params = LaunchConfiguration("kiss_params")
    projector_params = LaunchConfiguration("projector_params")

    default_kiss_params = PathJoinSubstitution([
        FindPackageShare("go2_kiss_icp_bringup"),
        "config",
        "go2_kiss_icp.yaml",
    ])
    default_projector_params = PathJoinSubstitution([
        FindPackageShare("go2_kiss_icp_bringup"),
        "config",
        "kiss_map_projector.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_base"),
        DeclareLaunchArgument("kiss_params", default_value=default_kiss_params),
        DeclareLaunchArgument("projector_params", default_value=default_projector_params),

        Node(
            package="kiss_icp",
            executable="kiss_icp_node",
            name="kiss_icp_node",
            output="screen",
            parameters=[kiss_params],
            remappings=[
                ("pointcloud_topic", cloud_topic),
            ],
        ),

        Node(
            package="go2_cloud_map_projector",
            executable="go2_cloud_map_projector",
            name="kiss_map_projector",
            output="screen",
            parameters=[projector_params],
        ),
    ])
