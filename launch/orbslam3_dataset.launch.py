from __future__ import annotations

import csv
import tempfile
from pathlib import Path

from launch import LaunchDescription
from launch.action import Action
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _normalize_name(value: str) -> str:
    return value.strip().lower().replace("-", "_")


def _is_mono_mode(mode: str) -> bool:
    return _normalize_name(mode) in ("mono", "monocular")


def _detect_dataset_family(dataset_root: Path) -> str:
    if (dataset_root / "mav0" / "cam0" / "data.csv").is_file():
        return "euroc"
    if (dataset_root / "image_0").is_dir() and (dataset_root / "times.txt").is_file():
        return "kitti"
    if (dataset_root / "associations.txt").is_file() or (dataset_root / "rgb.txt").is_file():
        return "tum"
    raise RuntimeError(f"Unable to detect dataset family from {dataset_root}")


def _resolve_dataset_family(explicit_family: str, dataset_root: Path | None) -> str:
    family = _normalize_name(explicit_family) if explicit_family else "auto"
    if family != "auto":
        return family
    if dataset_root is not None:
        return _detect_dataset_family(dataset_root)
    return "auto"


def _resolve_kitti_settings(package_share: Path, mode: str, dataset_root: Path) -> str:
    seq_name = dataset_root.name
    if not seq_name.isdigit():
        raise RuntimeError(
            f"KITTI dataset_root should point to a numeric sequence folder, got: {dataset_root}"
        )

    seq_num = int(seq_name)
    if seq_num <= 2:
        suffix = "00-02"
    elif seq_num == 3:
        suffix = "03"
    else:
        suffix = "04-12"

    if _is_mono_mode(mode):
        relative = Path(f"config/mono/KITTI{suffix}.yaml")
    elif _normalize_name(mode) == "stereo":
        relative = Path(f"config/stereo/KITTI{suffix}.yaml")
    else:
        raise RuntimeError(f"KITTI datasets support mono or stereo modes, got mode={mode}")

    return str(package_share / relative)


def _resolve_settings_file(package_share: Path, family: str, mode: str, explicit_settings: str,
                           dataset_root: Path | None) -> str:
    if explicit_settings:
        return explicit_settings

    mode_name = _normalize_name(mode)
    family_name = _normalize_name(family)

    if family_name == "auto":
        if _is_mono_mode(mode_name):
            return str(package_share / "config" / "mono" / "TUM1.yaml")
        if mode_name == "stereo":
            return str(package_share / "config" / "stereo" / "EuRoC.yaml")
        if mode_name == "stereo_inertial":
            return str(package_share / "config" / "stereo_inertial" / "EuRoC.yaml")
        return str(package_share / "config" / "rgbd" / "TUM1.yaml")

    if family_name == "tum":
        if _is_mono_mode(mode_name):
            return str(package_share / "config" / "mono" / "TUM1.yaml")
        if mode_name == "rgbd":
            return str(package_share / "config" / "rgbd" / "TUM1.yaml")
        raise RuntimeError("TUM datasets currently support mono or rgbd modes")

    if family_name == "euroc":
        if _is_mono_mode(mode_name):
            return str(package_share / "config" / "mono" / "EuRoC.yaml")
        if mode_name == "stereo":
            return str(package_share / "config" / "stereo" / "EuRoC.yaml")
        if mode_name == "stereo_inertial":
            return str(package_share / "config" / "stereo_inertial" / "EuRoC.yaml")
        raise RuntimeError("EuRoC datasets support mono, stereo, or stereo_inertial modes")

    if family_name == "kitti":
        if dataset_root is None:
            raise RuntimeError("KITTI dataset_root is required when settings_file is omitted")
        return _resolve_kitti_settings(package_share, mode_name, dataset_root)

    raise RuntimeError(f"Unsupported dataset family: {family}")


def _write_temp_file(prefix: str, lines: list[str]) -> Path:
    handle = tempfile.NamedTemporaryFile(mode="w", suffix=".txt", prefix=prefix, delete=False)
    try:
        handle.write("\n".join(lines))
        if lines:
            handle.write("\n")
    finally:
        handle.close()
    return Path(handle.name)


def _load_timestamped_entries(source_file: Path) -> list[tuple[float, str]]:
    entries: list[tuple[float, str]] = []
    with source_file.open() as stream:
        for line in stream:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue

            parts = stripped.split(maxsplit=1)
            if len(parts) != 2:
                continue

            try:
                timestamp = float(parts[0])
            except ValueError:
                continue

            entries.append((timestamp, parts[1]))

    return entries


def _build_tum_rgbd_association_lines(dataset_root: Path) -> list[str]:
    rgb_file = dataset_root / "rgb.txt"
    depth_file = dataset_root / "depth.txt"
    if not rgb_file.is_file() or not depth_file.is_file():
        raise RuntimeError(f"Missing TUM RGB-D source files under {dataset_root}")

    rgb_entries = _load_timestamped_entries(rgb_file)
    depth_entries = _load_timestamped_entries(depth_file)
    if not rgb_entries or not depth_entries:
        raise RuntimeError(f"Empty TUM RGB-D source files under {dataset_root}")

    association_lines: list[str] = []
    depth_index = 0
    max_delta_sec = 0.02

    for rgb_timestamp, rgb_path in rgb_entries:
        best_index = -1
        best_delta = float("inf")

        search_start = max(0, depth_index - 1)
        search_end = min(len(depth_entries), depth_index + 2)
        for index in range(search_start, search_end):
            depth_timestamp, _ = depth_entries[index]
            delta = abs(depth_timestamp - rgb_timestamp)
            if delta < best_delta:
                best_delta = delta
                best_index = index

        if best_index < 0 or best_delta > max_delta_sec:
            continue

        depth_timestamp, depth_path = depth_entries[best_index]
        association_lines.append(
            f"{rgb_timestamp:.9f} {(dataset_root / rgb_path).resolve()} "
            f"{depth_timestamp:.9f} {(dataset_root / depth_path).resolve()}"
        )
        depth_index = best_index + 1

    if not association_lines:
        raise RuntimeError(f"Failed to pair TUM RGB-D frames under {dataset_root}")

    return association_lines


def _read_tum_association_lines(dataset_root: Path, mode: str) -> tuple[Path, Path | None, list[Path]]:
    cleanup_paths: list[Path] = []
    mode_name = _normalize_name(mode)

    if _is_mono_mode(mode_name):
        source_file = dataset_root / "rgb.txt"
        if not source_file.is_file():
            raise RuntimeError(f"Missing TUM mono source file: {source_file}")
        return source_file, None, cleanup_paths

    if mode_name == "rgbd":
        source_file = dataset_root / "associations.txt"
        if source_file.is_file():
            return source_file, None, cleanup_paths

        association_lines = _build_tum_rgbd_association_lines(dataset_root)
        generated_file = _write_temp_file("orbslam3_ros_tum_assoc_", association_lines)
        cleanup_paths.append(generated_file)
        return generated_file, None, cleanup_paths

    raise RuntimeError("TUM datasets support mono or rgbd modes")


def _build_euroc_assets(dataset_root: Path, mode: str) -> tuple[Path, Path | None, list[Path]]:
    cleanup_paths: list[Path] = []
    cam0_rows: list[list[str]] = []
    cam1_rows: list[list[str]] = []

    cam0_csv = dataset_root / "mav0" / "cam0" / "data.csv"
    cam1_csv = dataset_root / "mav0" / "cam1" / "data.csv"
    imu_csv = dataset_root / "mav0" / "imu0" / "data.csv"
    if not cam0_csv.is_file() or not cam1_csv.is_file():
        raise RuntimeError(f"Missing EuRoC camera CSV files under {dataset_root}")

    with cam0_csv.open(newline="") as stream:
        reader = csv.reader(stream)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            cam0_rows.append(row)

    with cam1_csv.open(newline="") as stream:
        reader = csv.reader(stream)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            cam1_rows.append(row)

    if len(cam0_rows) != len(cam1_rows):
        raise RuntimeError("EuRoC cam0 and cam1 frame counts differ")

    association_lines: list[str] = []
    for left_row, right_row in zip(cam0_rows, cam1_rows):
        left_stamp = int(left_row[0])
        right_stamp = int(right_row[0])
        if left_stamp != right_stamp:
            raise RuntimeError("EuRoC cam0 and cam1 timestamps do not align")

        timestamp = f"{left_stamp / 1e9:.9f}"
        left_path = (dataset_root / "mav0" / "cam0" / "data" / left_row[1]).resolve()
        right_path = (dataset_root / "mav0" / "cam1" / "data" / right_row[1]).resolve()
        association_lines.append(f"{timestamp} {left_path} {timestamp} {right_path}")

    association_file = _write_temp_file("orbslam3_ros_euroc_assoc_", association_lines)
    cleanup_paths.append(association_file)

    imu_file: Path | None = None
    if _normalize_name(mode) == "stereo_inertial":
        if not imu_csv.is_file():
            raise RuntimeError(f"Missing EuRoC IMU CSV file: {imu_csv}")

        imu_lines: list[str] = []
        with imu_csv.open(newline="") as stream:
            reader = csv.reader(stream)
            for row in reader:
                if not row or row[0].startswith("#"):
                    continue
                timestamp = f"{int(row[0]) / 1e9:.9f}"
                wx, wy, wz = row[1], row[2], row[3]
                ax, ay, az = row[4], row[5], row[6]
                imu_lines.append(f"{timestamp} {ax} {ay} {az} {wx} {wy} {wz}")

        imu_file = _write_temp_file("orbslam3_ros_euroc_imu_", imu_lines)
        cleanup_paths.append(imu_file)

    return association_file, imu_file, cleanup_paths


def _build_kitti_assets(dataset_root: Path, mode: str) -> tuple[Path, Path | None, list[Path]]:
    cleanup_paths: list[Path] = []
    times_file = dataset_root / "times.txt"
    image_0 = dataset_root / "image_0"
    image_1 = dataset_root / "image_1"

    if not times_file.is_file() or not image_0.is_dir():
        raise RuntimeError(f"Missing KITTI sequence files under {dataset_root}")

    timestamps: list[str] = []
    with times_file.open() as stream:
        for line in stream:
            stripped = line.strip()
            if stripped:
                timestamps.append(stripped.split()[0])

    association_lines: list[str] = []
    for index, timestamp in enumerate(timestamps):
        left_path = (image_0 / f"{index:06d}.png").resolve()
        if _is_mono_mode(mode):
            association_lines.append(f"{timestamp} {left_path}")
            continue

        if _normalize_name(mode) != "stereo":
            raise RuntimeError("KITTI datasets support mono or stereo modes")

        if not image_1.is_dir():
            raise RuntimeError(f"Missing KITTI right image directory: {image_1}")
        right_path = (image_1 / f"{index:06d}.png").resolve()
        association_lines.append(f"{timestamp} {left_path} {timestamp} {right_path}")

    association_file = _write_temp_file("orbslam3_ros_kitti_assoc_", association_lines)
    cleanup_paths.append(association_file)
    return association_file, None, cleanup_paths


def _prepare_dataset_assets(dataset_family: str, dataset_root: Path | None, mode: str,
                            association_file: str, imu_file: str) -> tuple[str, str, list[Path]]:
    cleanup_paths: list[Path] = []
    family_name = _normalize_name(dataset_family)

    if association_file:
        association_path = Path(association_file).expanduser().resolve()
        return str(association_path), imu_file, cleanup_paths

    if family_name == "tum":
        if dataset_root is None:
            raise RuntimeError("TUM dataset_root is required when association_file is omitted")
        association_path, generated_imu, generated_cleanup = _read_tum_association_lines(dataset_root, mode)
        cleanup_paths.extend(generated_cleanup)
        return str(association_path), imu_file, cleanup_paths

    if family_name == "euroc":
        if dataset_root is None:
            raise RuntimeError("EuRoC dataset_root is required when association_file is omitted")
        association_path, generated_imu, generated_cleanup = _build_euroc_assets(dataset_root, mode)
        cleanup_paths.extend(generated_cleanup)
        if imu_file:
            return str(association_path), imu_file, cleanup_paths
        if _normalize_name(mode) == "stereo_inertial" and generated_imu is None:
            raise RuntimeError("EuRoC stereo_inertial playback requires an IMU file")
        return str(association_path), "" if generated_imu is None else str(generated_imu), cleanup_paths

    if family_name == "kitti":
        if dataset_root is None:
            raise RuntimeError("KITTI dataset_root is required when association_file is omitted")
        association_path, generated_imu, generated_cleanup = _build_kitti_assets(dataset_root, mode)
        cleanup_paths.extend(generated_cleanup)
        return str(association_path), imu_file, cleanup_paths

    if family_name == "auto":
        if dataset_root is not None and dataset_root.exists():
            detected = _detect_dataset_family(dataset_root)
            return _prepare_dataset_assets(detected, dataset_root, mode, association_file, imu_file)
        raise RuntimeError("dataset_family=auto requires dataset_root or an explicit association_file")

    raise RuntimeError(f"Unsupported dataset family: {dataset_family}")


def launch_setup(context, *args, **kwargs):
    package_share = Path(FindPackageShare("orbslam3_ros").perform(context))
    mode = LaunchConfiguration("mode").perform(context)
    dataset_family = LaunchConfiguration("dataset_family").perform(context)
    dataset_root_value = LaunchConfiguration("dataset_root").perform(context)
    dataset_root = Path(dataset_root_value).expanduser().resolve() if dataset_root_value else None
    association_file = LaunchConfiguration("association_file").perform(context)
    imu_file = LaunchConfiguration("imu_file").perform(context)
    resolved_family = _resolve_dataset_family(dataset_family, dataset_root)

    effective_settings_file = _resolve_settings_file(
        package_share,
        resolved_family,
        mode,
        LaunchConfiguration("settings_file").perform(context),
        dataset_root,
    )

    effective_dataset_root = dataset_root_value
    if not effective_dataset_root and association_file:
        effective_dataset_root = str(Path(association_file).expanduser().resolve().parent)

    resolved_association_file, resolved_imu_file, cleanup_paths = _prepare_dataset_assets(
        resolved_family,
        dataset_root,
        mode,
        association_file,
        imu_file,
    )

    node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orbslam3_ros"), "launch", "orbslam3.launch.py"]
            )
        ),
        launch_arguments={
            "mode": mode,
            "run_mode": "dataset",
            "dataset_root": effective_dataset_root,
            "node_name": LaunchConfiguration("node_name"),
            "vocab_file": LaunchConfiguration("vocab_file"),
            "settings_file": effective_settings_file,
            "use_viewer": LaunchConfiguration("use_viewer"),
            "association_file": resolved_association_file,
            "imu_file": resolved_imu_file,
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

    actions: list[Action] = [node]

    if cleanup_paths:
        def _cleanup_temp_files(context, *args, **kwargs):
            for path in cleanup_paths:
                try:
                    path.unlink()
                except FileNotFoundError:
                    pass
            return []

        actions.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        OpaqueFunction(function=_cleanup_temp_files),
                    ]
                )
            )
        )

    return actions


def generate_launch_description():
    default_vocab = PathJoinSubstitution(
        [FindPackageShare("orbslam3_ros"), "vocabulary", "ORBvoc.txt"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("dataset_family", default_value="auto"),
            DeclareLaunchArgument("dataset_root", default_value=""),
            DeclareLaunchArgument("mode", default_value="rgbd"),
            DeclareLaunchArgument("node_name", default_value="orbslam3_dataset_node"),
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
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("camera_optical_frame", default_value="camera_optical_frame"),
            OpaqueFunction(function=launch_setup),
        ]
    )
