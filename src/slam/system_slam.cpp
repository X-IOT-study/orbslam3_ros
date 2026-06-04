/**
 * @file system_slam.cpp
 * @brief Generic ORB-SLAM3 system wrapper implementation.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#include "orbslam3_ros/slam/system_slam.hpp"

#include <Eigen/Core>

namespace orbslam3_ros {

    SystemSlam::SystemSlam(
        const std::string& vocab_file,
        const std::string& settings_file,
        SensorMode sensor_mode,
        bool use_viewer,
        const std::string& sequence_name
    )
    : sensor_mode_(sensor_mode) {
        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocab_file,
            settings_file,
            ToSensorType(sensor_mode),
            use_viewer,
            0,
            sequence_name
        );
    }

    SystemSlam::~SystemSlam() {
        Shutdown();
    }

    bool SystemSlam::IsReady() const noexcept {
        return static_cast<bool>(slam_);
    }

    SensorMode SystemSlam::GetSensorMode() const noexcept {
        return sensor_mode_;
    }

    void SystemSlam::Shutdown() {
        if (!slam_) {
            return;
        }

        slam_->Shutdown();
        slam_.reset();
    }

    void SystemSlam::SaveTUMTrajectories(
        const std::filesystem::path& camera_trajectory_file,
        const std::filesystem::path& keyframe_trajectory_file
    ) {
        if (!slam_) {
            return;
        }

        slam_->Shutdown();
        slam_->SaveTrajectoryTUM(camera_trajectory_file.string());
        slam_->SaveKeyFrameTrajectoryTUM(keyframe_trajectory_file.string());
        slam_.reset();
    }

    void SystemSlam::SaveEuRoCTrajectories(
        const std::filesystem::path& camera_trajectory_file,
        const std::filesystem::path& keyframe_trajectory_file
    ) {
        if (!slam_) {
            return;
        }

        slam_->Shutdown();
        slam_->SaveTrajectoryEuRoC(camera_trajectory_file.string());
        slam_->SaveKeyFrameTrajectoryEuRoC(keyframe_trajectory_file.string());
        slam_.reset();
    }

    std::optional<Sophus::SE3f> SystemSlam::TrackMonocular(
        const cv::Mat& image,
        double timestamp,
        const std::vector<ORB_SLAM3::IMU::Point>& imu_measurements,
        const std::string& filename
    ) {
        if (!IsReady() || image.empty()) {
            return std::nullopt;
        }

        return slam_->TrackMonocular(image, timestamp, imu_measurements, filename);
    }

    std::optional<Sophus::SE3f> SystemSlam::TrackStereo(
        const cv::Mat& left_image,
        const cv::Mat& right_image,
        double timestamp,
        const std::vector<ORB_SLAM3::IMU::Point>& imu_measurements,
        const std::string& filename
    ) {
        if (!IsReady() || left_image.empty() || right_image.empty()) {
            return std::nullopt;
        }

        return slam_->TrackStereo(left_image, right_image, timestamp, imu_measurements, filename);
    }

    std::optional<Sophus::SE3f> SystemSlam::TrackRgbd(
        const cv::Mat& image,
        const cv::Mat& depth_image,
        double timestamp,
        const std::vector<ORB_SLAM3::IMU::Point>& imu_measurements,
        const std::string& filename
    ) {
        if (!IsReady() || image.empty() || depth_image.empty()) {
            return std::nullopt;
        }

        return slam_->TrackRGBD(image, depth_image, timestamp, imu_measurements, filename);
    }

    TrackingState SystemSlam::GetTrackingState() {
        if (!IsReady()) {
            return TrackingState::Lost;
        }

        switch (slam_->GetTrackingState()) {
            case 1:
                return TrackingState::Initializing;
            case 2:
            case 5:
                return TrackingState::Tracking;
            default:
                return TrackingState::Lost;
        }
    }

    std::vector<MapPointSnapshot> SystemSlam::GetMapPointsSnapshot() {
        std::vector<MapPointSnapshot> map_points;
        if (!IsReady()) {
            return map_points;
        }

        const auto tracked_map_points = slam_->GetTrackedMapPoints();
        map_points.reserve(tracked_map_points.size());

        for (auto* map_point : tracked_map_points) {
            if (!map_point || map_point->isBad()) {
                continue;
            }

            const Eigen::Vector3f world_position = map_point->GetWorldPos();
            map_points.push_back(MapPointSnapshot{
                world_position.x(),
                world_position.y(),
                world_position.z()
            });
        }

        return map_points;
    }

    ORB_SLAM3::System::eSensor SystemSlam::ToSensorType(SensorMode sensor_mode) noexcept {
        switch (sensor_mode) {
            case SensorMode::Monocular:
                return ORB_SLAM3::System::MONOCULAR;
            case SensorMode::Stereo:
                return ORB_SLAM3::System::STEREO;
            case SensorMode::Rgbd:
                return ORB_SLAM3::System::RGBD;
            case SensorMode::StereoInertial:
                return ORB_SLAM3::System::IMU_STEREO;
        }

        return ORB_SLAM3::System::RGBD;
    }

}  // namespace orbslam3_ros
