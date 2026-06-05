from pathlib import Path
from typing import List

from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _resolve_settings_file(context):
    settings_file = LaunchConfiguration("settings_file").perform(context)
    if settings_file:
        return settings_file

    mode = LaunchConfiguration("mode").perform(context).strip().lower().replace("-", "_")
    package_share = Path(FindPackageShare("orbslam3_ros").perform(context))

    if mode in ("mono", "monocular"):
        relative_path = Path("config/mono/TUM1.yaml")
    elif mode == "stereo":
        relative_path = Path("config/stereo/EuRoC.yaml")
    elif mode in ("stereo_inertial", "stereoimu"):
        relative_path = Path("config/stereo_inertial/EuRoC.yaml")
    else:
        relative_path = Path("config/rgbd/astra_pro.yaml")

    return str(package_share / relative_path)


def launch_setup(context, *args, **kwargs):
    effective_settings_file = _resolve_settings_file(context)

    node = Node(
        package="orbslam3_ros",
        executable="orbslam3_ros_multimode_node",
        name=LaunchConfiguration("node_name"),
        output="screen",
        parameters=[
            {
                "mode": LaunchConfiguration("mode"),
                "run_mode": LaunchConfiguration("run_mode"),
                "dataset_root": LaunchConfiguration("dataset_root"),
                "vocab_file": LaunchConfiguration("vocab_file"),
                "settings_file": effective_settings_file,
                "use_viewer": ParameterValue(LaunchConfiguration("use_viewer"), value_type=bool),
                "image_topic": LaunchConfiguration("image_topic"),
                "left_topic": LaunchConfiguration("left_topic"),
                "right_topic": LaunchConfiguration("right_topic"),
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "image_transport": LaunchConfiguration("image_transport"),
                "left_transport": LaunchConfiguration("left_transport"),
                "right_transport": LaunchConfiguration("right_transport"),
                "rgb_transport": LaunchConfiguration("rgb_transport"),
                "depth_transport": LaunchConfiguration("depth_transport"),
                "rgb_camera_info_topic": LaunchConfiguration("rgb_camera_info_topic"),
                "runtime_calibration_from_camera_info": ParameterValue(
                    LaunchConfiguration("runtime_calibration_from_camera_info"),
                    value_type=bool,
                ),
                "association_file": LaunchConfiguration("association_file"),
                "imu_file": LaunchConfiguration("imu_file"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "path_topic": LaunchConfiguration("path_topic"),
                "tracking_state_topic": LaunchConfiguration("tracking_state_topic"),
                "map_points_topic": LaunchConfiguration("map_points_topic"),
                "map_frame": LaunchConfiguration("map_frame"),
                "world_frame": LaunchConfiguration("world_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "camera_optical_frame": LaunchConfiguration("camera_optical_frame"),
                "queue_size": ParameterValue(LaunchConfiguration("queue_size"), value_type=int),
                "sync_queue_size": ParameterValue(LaunchConfiguration("sync_queue_size"), value_type=int),
                "frame_queue_size": ParameterValue(LaunchConfiguration("frame_queue_size"), value_type=int),
                "path_history_size": ParameterValue(LaunchConfiguration("path_history_size"), value_type=int),
                "path_update_distance": ParameterValue(LaunchConfiguration("path_update_distance"), value_type=float),
                "path_update_interval": ParameterValue(LaunchConfiguration("path_update_interval"), value_type=float),
                "map_points_rate": ParameterValue(LaunchConfiguration("map_points_rate"), value_type=float),
                "sync_tolerance_sec": ParameterValue(LaunchConfiguration("sync_tolerance_sec"), value_type=float),
                "stats_hz": ParameterValue(LaunchConfiguration("stats_hz"), value_type=float),
                "diagnostic_logging": ParameterValue(LaunchConfiguration("diagnostic_logging"), value_type=bool),
                "publish_tf": ParameterValue(LaunchConfiguration("publish_tf"), value_type=bool),
                "publish_static_camera_tf": ParameterValue(
                    LaunchConfiguration("publish_static_camera_tf"),
                    value_type=bool,
                ),
                "publish_static_optical_tf": ParameterValue(
                    LaunchConfiguration("publish_static_optical_tf"),
                    value_type=bool,
                ),
                "invert_pose": ParameterValue(LaunchConfiguration("invert_pose"), value_type=bool),
                "loop": ParameterValue(LaunchConfiguration("loop"), value_type=bool),
                "playback_rate": ParameterValue(LaunchConfiguration("playback_rate"), value_type=float),
            }
        ],
    )

    actions: List[Action] = [node]

    dataset_source = LaunchConfiguration("dataset_source").perform(context)
    run_mode = LaunchConfiguration("run_mode").perform(context)
    bag_path = LaunchConfiguration("bag_path").perform(context)
    if dataset_source == "bag" and run_mode == "realtime" and bag_path:
        bag_read_ahead_queue_size = LaunchConfiguration("bag_read_ahead_queue_size")
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    LaunchConfiguration("bag_path"),
                    "--clock",
                    "--read-ahead-queue-size",
                    bag_read_ahead_queue_size,
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    default_vocab = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "vocabulary", "ORBvoc.txt"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="rgbd"),
            DeclareLaunchArgument("run_mode", default_value="realtime"),
            DeclareLaunchArgument("dataset_source", default_value="sequence"),
            DeclareLaunchArgument("dataset_root", default_value=""),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("bag_read_ahead_queue_size", default_value="1000"),
            DeclareLaunchArgument(
                "node_name",
                default_value="orbslam3_multimode_node",
            ),
            DeclareLaunchArgument("vocab_file", default_value=default_vocab),
            DeclareLaunchArgument("settings_file", default_value=""),
            DeclareLaunchArgument("use_viewer", default_value="true"),
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("left_topic", default_value="/camera/left/image_raw"),
            DeclareLaunchArgument("right_topic", default_value="/camera/right/image_raw"),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("imu_topic", default_value="/camera/imu"),
            DeclareLaunchArgument("image_transport", default_value="raw"),
            DeclareLaunchArgument("left_transport", default_value="raw"),
            DeclareLaunchArgument("right_transport", default_value="raw"),
            DeclareLaunchArgument("rgb_transport", default_value="raw"),
            DeclareLaunchArgument("depth_transport", default_value="raw"),
            DeclareLaunchArgument("rgb_camera_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument(
                "runtime_calibration_from_camera_info",
                default_value="true",
            ),
            DeclareLaunchArgument("association_file", default_value=""),
            DeclareLaunchArgument("imu_file", default_value=""),
            DeclareLaunchArgument("pose_topic", default_value="/orbslam3/pose"),
            DeclareLaunchArgument("odom_topic", default_value="/orbslam3/odom"),
            DeclareLaunchArgument("path_topic", default_value="/orbslam3/path"),
            DeclareLaunchArgument(
                "tracking_state_topic",
                default_value="/orbslam3/tracking_state",
            ),
            DeclareLaunchArgument("map_points_topic", default_value="/orbslam3/map_points"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("world_frame", default_value="map"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument(
                "camera_optical_frame",
                default_value="camera_optical_frame",
            ),
            DeclareLaunchArgument("queue_size", default_value="10"),
            DeclareLaunchArgument("sync_queue_size", default_value="10"),
            DeclareLaunchArgument("frame_queue_size", default_value="10"),
            DeclareLaunchArgument("path_history_size", default_value="200"),
            DeclareLaunchArgument("path_update_distance", default_value="0.05"),
            DeclareLaunchArgument("path_update_interval", default_value="0.1"),
            DeclareLaunchArgument("map_points_rate", default_value="30.0"),
            DeclareLaunchArgument("sync_tolerance_sec", default_value="0.05"),
            DeclareLaunchArgument("stats_hz", default_value="1.0"),
            DeclareLaunchArgument("diagnostic_logging", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_static_camera_tf", default_value="false"),
            DeclareLaunchArgument("publish_static_optical_tf", default_value="false"),
            DeclareLaunchArgument("invert_pose", default_value="false"),
            DeclareLaunchArgument("loop", default_value="false"),
            DeclareLaunchArgument("playback_rate", default_value="1.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
