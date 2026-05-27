from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    pointcloud_params = LaunchConfiguration("pointcloud_params")
    slam_params = LaunchConfiguration("slam_params")

    default_pointcloud_params = PathJoinSubstitution([
        FindPackageShare("go2_slam_bringup"),
        "config",
        "pointcloud_to_laserscan.yaml",
    ])
    default_slam_params = PathJoinSubstitution([
        FindPackageShare("go2_slam_bringup"),
        "config",
        "slam_toolbox.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_base"),
        DeclareLaunchArgument("odom_topic", default_value="/utlidar/robot_odom"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("pointcloud_params", default_value=default_pointcloud_params),
        DeclareLaunchArgument("slam_params", default_value=default_slam_params),

        Node(
            package="go2_tf_bridge",
            executable="go2_tf_bridge",
            name="go2_tf_bridge",
            output="screen",
            parameters=[{"odom_topic": odom_topic}],
        ),

        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            parameters=[pointcloud_params],
            remappings=[
                ("cloud_in", cloud_topic),
                ("scan", scan_topic),
            ],
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
            remappings=[
                ("scan", scan_topic),
            ],
        ),
    ])
