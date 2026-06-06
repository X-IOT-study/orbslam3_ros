from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _read_text_file(path: str) -> str:
    return Path(path).expanduser().read_text(encoding="utf-8")


def _resolve_default_settings(context) -> str:
    settings_file = LaunchConfiguration("settings_file").perform(context)
    if settings_file:
        return settings_file
    return str(
        Path(FindPackageShare("orbslam3_ros").perform(context))
        / "config"
        / "rgbd"
        / "astra_pro.yaml"
    )


def launch_setup(context, *args, **kwargs):
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    robot_description = _read_text_file(urdf_file)
    settings_file = _resolve_default_settings(context)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    astra_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("astra_camera"), "launch", "astra_pro.launch.xml"]
            )
        ),
        launch_arguments={
            "camera_name": LaunchConfiguration("camera_name"),
            "publish_tf": LaunchConfiguration("astra_publish_tf"),
            "depth_registration": LaunchConfiguration("depth_registration"),
        }.items(),
    )

    orbslam3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("orbslam3_ros"), "launch", "orbslam3.launch.py"])
        ),
        launch_arguments={
            "mode": "rgbd",
            "run_mode": "realtime",
            "node_name": LaunchConfiguration("node_name"),
            "vocab_file": LaunchConfiguration("vocab_file"),
            "settings_file": settings_file,
            "use_viewer": LaunchConfiguration("use_viewer"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
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
            "publish_tf": LaunchConfiguration("publish_tf"),
            "publish_static_camera_tf": LaunchConfiguration("publish_static_camera_tf"),
            "publish_static_optical_tf": LaunchConfiguration("publish_static_optical_tf"),
            "base_frame": LaunchConfiguration("base_frame"),
            "camera_frame": LaunchConfiguration("camera_frame"),
        }.items(),
    )

    return [robot_state_publisher, astra_launch, orbslam3_launch]


def generate_launch_description():
    default_vocab = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "vocabulary", "ORBvoc.txt"]
    )
    default_urdf = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "urdf", "minimal_camera_test.urdf"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("node_name", default_value="orbslam3_astra_test_node"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument("urdf_file", default_value=default_urdf),
            DeclareLaunchArgument("vocab_file", default_value=default_vocab),
            DeclareLaunchArgument(
                "settings_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("orbslam3_ros"), "config", "rgbd", "astra_pro.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_viewer", default_value="false"),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("rgb_camera_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("image_transport", default_value="raw"),
            DeclareLaunchArgument("depth_transport", default_value="raw"),
            DeclareLaunchArgument("runtime_calibration_from_camera_info", default_value="true"),
            DeclareLaunchArgument("queue_size", default_value="10"),
            DeclareLaunchArgument("sync_queue_size", default_value="10"),
            DeclareLaunchArgument("frame_queue_size", default_value="10"),
            DeclareLaunchArgument("sync_tolerance_sec", default_value="0.05"),
            DeclareLaunchArgument("stats_hz", default_value="1.0"),
            DeclareLaunchArgument("map_points_rate", default_value="30.0"),
            DeclareLaunchArgument("diagnostic_logging", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_static_camera_tf", default_value="false"),
            DeclareLaunchArgument("publish_static_optical_tf", default_value="false"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("astra_publish_tf", default_value="true"),
            DeclareLaunchArgument("depth_registration", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
