from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _read_text_file(path: str) -> str:
    return Path(path).expanduser().read_text(encoding="utf-8")


def launch_setup(context, *args, **kwargs):
    urdf_file = LaunchConfiguration("urdf_file").perform(context)
    robot_description = _read_text_file(urdf_file)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    dataset_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orbslam3_ros"), "launch", "orbslam3_dataset.launch.py"]
            )
        ),
        launch_arguments={
            "dataset_family": LaunchConfiguration("dataset_family"),
            "dataset_root": LaunchConfiguration("dataset_root"),
            "mode": LaunchConfiguration("mode"),
            "node_name": LaunchConfiguration("node_name"),
            "vocab_file": LaunchConfiguration("vocab_file"),
            "settings_file": LaunchConfiguration("settings_file"),
            "use_viewer": LaunchConfiguration("use_viewer"),
            "association_file": LaunchConfiguration("association_file"),
            "imu_file": LaunchConfiguration("imu_file"),
            "queue_size": LaunchConfiguration("queue_size"),
            "loop": LaunchConfiguration("loop"),
            "playback_rate": LaunchConfiguration("playback_rate"),
            "map_points_rate": LaunchConfiguration("map_points_rate"),
            "publish_tf": LaunchConfiguration("publish_tf"),
            "publish_static_camera_tf": LaunchConfiguration("publish_static_camera_tf"),
            "publish_static_optical_tf": LaunchConfiguration("publish_static_optical_tf"),
            "base_frame": LaunchConfiguration("base_frame"),
            "camera_frame": LaunchConfiguration("camera_frame"),
            "camera_optical_frame": LaunchConfiguration("camera_optical_frame"),
        }.items(),
    )

    return [robot_state_publisher, dataset_launch]


def generate_launch_description():
    default_vocab = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "vocabulary", "ORBvoc.txt"]
    )
    default_urdf = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "urdf", "minimal_dataset_camera_test.urdf"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_family", default_value="auto"),
            DeclareLaunchArgument("dataset_root", default_value=""),
            DeclareLaunchArgument("mode", default_value="rgbd"),
            DeclareLaunchArgument("node_name", default_value="orbslam3_dataset_bodyframe_node"),
            DeclareLaunchArgument("vocab_file", default_value=default_vocab),
            DeclareLaunchArgument("settings_file", default_value=""),
            DeclareLaunchArgument("use_viewer", default_value="true"),
            DeclareLaunchArgument("association_file", default_value=""),
            DeclareLaunchArgument("imu_file", default_value=""),
            DeclareLaunchArgument("queue_size", default_value="10"),
            DeclareLaunchArgument("loop", default_value="false"),
            DeclareLaunchArgument("playback_rate", default_value="1.0"),
            DeclareLaunchArgument("map_points_rate", default_value="30.0"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument("publish_static_camera_tf", default_value="false"),
            DeclareLaunchArgument("publish_static_optical_tf", default_value="false"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_frame", default_value="camera_optical_frame"),
            DeclareLaunchArgument("camera_optical_frame", default_value="camera_optical_frame"),
            DeclareLaunchArgument("urdf_file", default_value=default_urdf),
            OpaqueFunction(function=launch_setup),
        ]
    )
