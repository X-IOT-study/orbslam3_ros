from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_vocab = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "vocabulary", "ORBvoc.txt"]
    )
    default_settings = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "config", "TUM1.yaml"]
    )

    mode = LaunchConfiguration("mode")
    vocab_file = LaunchConfiguration("vocab_file")
    settings_file = LaunchConfiguration("settings_file")
    use_viewer = LaunchConfiguration("use_viewer")
    association_file = LaunchConfiguration("association_file")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    image_transport = LaunchConfiguration("image_transport")
    depth_transport = LaunchConfiguration("depth_transport")
    rgb_camera_info_topic = LaunchConfiguration("rgb_camera_info_topic")
    pose_topic = LaunchConfiguration("pose_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    path_topic = LaunchConfiguration("path_topic")
    tracking_state_topic = LaunchConfiguration("tracking_state_topic")
    map_points_topic = LaunchConfiguration("map_points_topic")
    map_frame = LaunchConfiguration("map_frame")
    world_frame = LaunchConfiguration("world_frame")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    camera_optical_frame = LaunchConfiguration("camera_optical_frame")
    queue_size = LaunchConfiguration("queue_size")
    sync_queue_size = LaunchConfiguration("sync_queue_size")
    frame_queue_size = LaunchConfiguration("frame_queue_size")
    path_history_size = LaunchConfiguration("path_history_size")
    path_update_distance = LaunchConfiguration("path_update_distance")
    path_update_interval = LaunchConfiguration("path_update_interval")
    map_points_rate = LaunchConfiguration("map_points_rate")
    sync_tolerance_sec = LaunchConfiguration("sync_tolerance_sec")
    stats_hz = LaunchConfiguration("stats_hz")
    diagnostic_logging = LaunchConfiguration("diagnostic_logging")
    publish_tf = LaunchConfiguration("publish_tf")
    publish_static_camera_tf = LaunchConfiguration("publish_static_camera_tf")
    publish_static_optical_tf = LaunchConfiguration("publish_static_optical_tf")
    invert_pose = LaunchConfiguration("invert_pose")
    loop = LaunchConfiguration("loop")
    playback_rate = LaunchConfiguration("playback_rate")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="realtime",
                description="Select 'realtime' or 'dataset' node mode.",
            ),
            DeclareLaunchArgument(
                "vocab_file",
                default_value=default_vocab,
                description="Path to the ORB vocabulary file.",
            ),
            DeclareLaunchArgument(
                "settings_file",
                default_value=default_settings,
                description="Path to the ORB-SLAM3 settings file.",
            ),
            DeclareLaunchArgument(
                "use_viewer",
                default_value="true",
                description="Enable the ORB-SLAM3 viewer.",
            ),
            DeclareLaunchArgument(
                "association_file",
                default_value="",
                description="Path to the TUM association file for dataset mode.",
            ),
            DeclareLaunchArgument(
                "rgb_topic",
                default_value="/camera/color/image_raw",
                description="RGB image base topic for realtime mode.",
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/depth/image_raw",
                description="Depth image base topic for realtime mode.",
            ),
            DeclareLaunchArgument(
                "image_transport",
                default_value="raw",
                description="RGB image_transport plugin (raw, compressed, etc.).",
            ),
            DeclareLaunchArgument(
                "depth_transport",
                default_value="raw",
                description="Depth image_transport plugin (raw, compressedDepth, etc.).",
            ),
            DeclareLaunchArgument(
                "rgb_camera_info_topic",
                default_value="/camera/color/camera_info",
                description="RGB camera info topic for realtime mode.",
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value="/orbslam3/pose",
                description="Pose output topic.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/orbslam3/odom",
                description="Odometry output topic.",
            ),
            DeclareLaunchArgument(
                "path_topic",
                default_value="/orbslam3/path",
                description="Path output topic.",
            ),
            DeclareLaunchArgument(
                "tracking_state_topic",
                default_value="/orbslam3/tracking_state",
                description="Tracking state output topic.",
            ),
            DeclareLaunchArgument(
                "map_points_topic",
                default_value="/orbslam3/map_points",
                description="Map points output topic.",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Map frame used for pose and TF output.",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value="map",
                description="World frame used for pose and TF output.",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_link",
                description="Robot base frame used for TF and odometry output.",
            ),
            DeclareLaunchArgument(
                "camera_frame",
                default_value="camera_link",
                description="Camera frame used for TF output.",
            ),
            DeclareLaunchArgument(
                "camera_optical_frame",
                default_value="camera_optical_frame",
                description="Optical frame used for the static camera_frame -> optical TF.",
            ),
            DeclareLaunchArgument(
                "queue_size",
                default_value="10",
                description="Legacy compatibility queue size.",
            ),
            DeclareLaunchArgument(
                "sync_queue_size",
                default_value="10",
                description="Approximate-time synchronizer queue size.",
            ),
            DeclareLaunchArgument(
                "frame_queue_size",
                default_value="10",
                description="Bounded SPSC frame queue size.",
            ),
            DeclareLaunchArgument(
                "path_history_size",
                default_value="200",
                description="Maximum number of path samples to retain.",
            ),
            DeclareLaunchArgument(
                "path_update_distance",
                default_value="0.05",
                description="Minimum translation needed before a path point is appended.",
            ),
            DeclareLaunchArgument(
                "path_update_interval",
                default_value="0.1",
                description="Maximum path sampling interval in seconds.",
            ),
            DeclareLaunchArgument(
                "map_points_rate",
                default_value="30.0",
                description="Map points publish frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "sync_tolerance_sec",
                default_value="0.05",
                description="RGB-D pairing tolerance in seconds.",
            ),
            DeclareLaunchArgument(
                "stats_hz",
                default_value="1.0",
                description="Diagnostic log frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "diagnostic_logging",
                default_value="false",
                description="Enable verbose node-side diagnostics.",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description="Publish TF output.",
            ),
            DeclareLaunchArgument(
                "publish_static_camera_tf",
                default_value="false",
                description="Publish a fallback static base->camera transform.",
            ),
            DeclareLaunchArgument(
                "publish_static_optical_tf",
                default_value="false",
                description="Publish the camera_link -> camera_optical_frame static TF.",
            ),
            DeclareLaunchArgument(
                "invert_pose",
                default_value="false",
                description="Invert the pose before publishing.",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                description="Loop dataset playback.",
            ),
            DeclareLaunchArgument(
                "playback_rate",
                default_value="1.0",
                description="Playback speed multiplier applied to dataset timestamps.",
            ),
            Node(
                package="orbslam3_ros",
                executable="orbslam3_ros_realtime_node",
                name="orbslam3_realtime_node",
                output="screen",
                condition=IfCondition(PythonExpression(["'", mode, "' == 'realtime'"])),
                parameters=[
                    {
                        "vocab_file": vocab_file,
                        "settings_file": settings_file,
                        "rgb_topic": rgb_topic,
                        "depth_topic": depth_topic,
                        "image_transport": image_transport,
                        "depth_transport": depth_transport,
                        "rgb_camera_info_topic": rgb_camera_info_topic,
                        "pose_topic": pose_topic,
                        "odom_topic": odom_topic,
                        "path_topic": path_topic,
                        "tracking_state_topic": tracking_state_topic,
                        "map_points_topic": map_points_topic,
                        "map_frame": map_frame,
                        "world_frame": world_frame,
                        "base_frame": base_frame,
                        "camera_frame": camera_frame,
                        "camera_optical_frame": camera_optical_frame,
                        "queue_size": ParameterValue(queue_size, value_type=int),
                        "sync_queue_size": ParameterValue(sync_queue_size, value_type=int),
                        "frame_queue_size": ParameterValue(frame_queue_size, value_type=int),
                        "path_history_size": ParameterValue(path_history_size, value_type=int),
                        "path_update_distance": ParameterValue(path_update_distance, value_type=float),
                        "path_update_interval": ParameterValue(path_update_interval, value_type=float),
                        "map_points_rate": ParameterValue(map_points_rate, value_type=float),
                        "sync_tolerance_sec": ParameterValue(sync_tolerance_sec, value_type=float),
                        "stats_hz": ParameterValue(stats_hz, value_type=float),
                        "diagnostic_logging": ParameterValue(diagnostic_logging, value_type=bool),
                        "publish_tf": ParameterValue(publish_tf, value_type=bool),
                        "publish_static_camera_tf": ParameterValue(
                            publish_static_camera_tf,
                            value_type=bool,
                        ),
                        "publish_static_optical_tf": ParameterValue(
                            publish_static_optical_tf,
                            value_type=bool,
                        ),
                        "invert_pose": ParameterValue(invert_pose, value_type=bool),
                        "use_viewer": ParameterValue(use_viewer, value_type=bool),
                    }
                ],
            ),
            Node(
                package="orbslam3_ros",
                executable="orbslam3_ros_dataset_node",
                name="orbslam3_dataset_node",
                output="screen",
                condition=IfCondition(PythonExpression(["'", mode, "' == 'dataset'"])),
                parameters=[
                    {
                        "vocab_file": vocab_file,
                        "settings_file": settings_file,
                        "association_file": association_file,
                        "pose_topic": pose_topic,
                        "odom_topic": odom_topic,
                        "path_topic": path_topic,
                        "tracking_state_topic": tracking_state_topic,
                        "map_points_topic": map_points_topic,
                        "map_frame": map_frame,
                        "world_frame": world_frame,
                        "base_frame": base_frame,
                        "camera_frame": camera_frame,
                        "queue_size": ParameterValue(queue_size, value_type=int),
                        "playback_rate": ParameterValue(playback_rate, value_type=float),
                        "path_history_size": ParameterValue(path_history_size, value_type=int),
                        "path_update_distance": ParameterValue(path_update_distance, value_type=float),
                        "path_update_interval": ParameterValue(path_update_interval, value_type=float),
                        "map_points_rate": ParameterValue(map_points_rate, value_type=float),
                        "loop": ParameterValue(loop, value_type=bool),
                        "publish_tf": ParameterValue(publish_tf, value_type=bool),
                        "publish_static_camera_tf": ParameterValue(
                            publish_static_camera_tf,
                            value_type=bool,
                        ),
                        "publish_static_optical_tf": ParameterValue(
                            publish_static_optical_tf,
                            value_type=bool,
                        ),
                        "invert_pose": ParameterValue(invert_pose, value_type=bool),
                        "use_viewer": ParameterValue(use_viewer, value_type=bool),
                    }
                ],
            ),
        ]
    )
