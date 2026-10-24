from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    alignment_mode = LaunchConfiguration("alignment_mode")
    fixed_offset_seconds = LaunchConfiguration("fixed_offset_seconds")
    aligned_cloud_topic = LaunchConfiguration("aligned_cloud_topic")
    aligned_odom_topic = LaunchConfiguration("aligned_odom_topic")
    accumulate_scan = LaunchConfiguration("accumulate_scan")
    accumulation_window = LaunchConfiguration("accumulation_window")
    accumulated_scan_params = LaunchConfiguration("accumulated_scan_params")

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
    default_accumulated_scan_params = PathJoinSubstitution([
        FindPackageShare("go2_slam_bringup"),
        "config",
        "accumulated_scan.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/utlidar/cloud_base"),
        DeclareLaunchArgument("odom_topic", default_value="/utlidar/robot_odom"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("pointcloud_params", default_value=default_pointcloud_params),
        DeclareLaunchArgument("slam_params", default_value=default_slam_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("alignment_mode", default_value="auto"),
        DeclareLaunchArgument("fixed_offset_seconds", default_value="0.0"),
        DeclareLaunchArgument("aligned_cloud_topic", default_value="/go2/slam/cloud_base"),
        DeclareLaunchArgument("aligned_odom_topic", default_value="/go2/slam/odom"),
        DeclareLaunchArgument("accumulate_scan", default_value="true"),
        DeclareLaunchArgument("accumulation_window", default_value="1.0"),
        DeclareLaunchArgument(
            "accumulated_scan_params", default_value=default_accumulated_scan_params),

        Node(
            package="go2_slam_bringup",
            executable="go2_sensor_time_bridge.py",
            name="go2_sensor_time_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "cloud_topic": cloud_topic,
                "odom_topic": odom_topic,
                "aligned_cloud_topic": aligned_cloud_topic,
                "aligned_odom_topic": aligned_odom_topic,
                "alignment_mode": alignment_mode,
                "fixed_offset_seconds": fixed_offset_seconds,
            }],
        ),

        Node(
            package="go2_tf_bridge",
            executable="go2_tf_bridge",
            name="go2_tf_bridge",
            output="screen",
            parameters=[{"odom_topic": aligned_odom_topic, "use_sim_time": use_sim_time}],
        ),

        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            condition=UnlessCondition(accumulate_scan),
            output="screen",
            parameters=[pointcloud_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("cloud_in", aligned_cloud_topic),
                ("scan", scan_topic),
            ],
        ),

        Node(
            package="go2_slam_bringup",
            executable="go2_accumulated_scan.py",
            name="go2_accumulated_scan",
            condition=IfCondition(accumulate_scan),
            output="screen",
            parameters=[accumulated_scan_params, {
                "use_sim_time": use_sim_time,
                "cloud_topic": aligned_cloud_topic,
                "odom_topic": aligned_odom_topic,
                "scan_topic": scan_topic,
                "window_seconds": accumulation_window,
            }],
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("scan", scan_topic),
            ],
        ),
    ])
