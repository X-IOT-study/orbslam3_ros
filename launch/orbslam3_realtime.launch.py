from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_vocab = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "vocabulary", "ORBvoc.txt"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="rgbd"),
            DeclareLaunchArgument("node_name", default_value="orbslam3_realtime_node"),
            DeclareLaunchArgument("vocab_file", default_value=default_vocab),
            DeclareLaunchArgument("settings_file", default_value=""),
            DeclareLaunchArgument("use_viewer", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("left_topic", default_value="/camera/left/image_raw"),
            DeclareLaunchArgument("right_topic", default_value="/camera/right/image_raw"),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("imu_topic", default_value="/camera/imu"),
            DeclareLaunchArgument("rgb_camera_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("image_transport", default_value="raw"),
            DeclareLaunchArgument("left_transport", default_value="raw"),
            DeclareLaunchArgument("right_transport", default_value="raw"),
            DeclareLaunchArgument("rgb_transport", default_value="raw"),
            DeclareLaunchArgument("depth_transport", default_value="raw"),
            DeclareLaunchArgument("runtime_calibration_from_camera_info", default_value="true"),
            DeclareLaunchArgument("queue_size", default_value="10"),
            DeclareLaunchArgument("sync_queue_size", default_value="10"),
            DeclareLaunchArgument("frame_queue_size", default_value="10"),
            DeclareLaunchArgument("sync_tolerance_sec", default_value="0.05"),
            DeclareLaunchArgument("stats_hz", default_value="1.0"),
            DeclareLaunchArgument("map_points_rate", default_value="30.0"),
            DeclareLaunchArgument("diagnostic_logging", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("orbslam3_ros"), "launch", "orbslam3.launch.py"]
                    )
                ),
                launch_arguments={
                    "mode": LaunchConfiguration("mode"),
                    "run_mode": "realtime",
                    "node_name": LaunchConfiguration("node_name"),
                    "vocab_file": LaunchConfiguration("vocab_file"),
                    "settings_file": LaunchConfiguration("settings_file"),
                    "use_viewer": LaunchConfiguration("use_viewer"),
                    "image_topic": LaunchConfiguration("image_topic"),
                    "left_topic": LaunchConfiguration("left_topic"),
                    "right_topic": LaunchConfiguration("right_topic"),
                    "rgb_topic": LaunchConfiguration("rgb_topic"),
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "imu_topic": LaunchConfiguration("imu_topic"),
                    "left_transport": LaunchConfiguration("left_transport"),
                    "right_transport": LaunchConfiguration("right_transport"),
                    "rgb_transport": LaunchConfiguration("rgb_transport"),
                    "rgb_camera_info_topic": LaunchConfiguration("rgb_camera_info_topic"),
                    "image_transport": LaunchConfiguration("image_transport"),
                    "depth_transport": LaunchConfiguration("depth_transport"),
                    "runtime_calibration_from_camera_info": LaunchConfiguration(
                        "runtime_calibration_from_camera_info"
                    ),
                    "queue_size": LaunchConfiguration("queue_size"),
                    "sync_queue_size": LaunchConfiguration("sync_queue_size"),
                    "frame_queue_size": LaunchConfiguration("frame_queue_size"),
                    "sync_tolerance_sec": LaunchConfiguration("sync_tolerance_sec"),
                    "stats_hz": LaunchConfiguration("stats_hz"),
                    "map_points_rate": LaunchConfiguration("map_points_rate"),
                    "diagnostic_logging": LaunchConfiguration("diagnostic_logging"),
                }.items(),
            )
        ]
    )
