/**
 * @file system_slam.hpp
 * @brief Generic ORB-SLAM3 system wrapper.
 * @author WenSheng Xu
 * @date 2026-06-04
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_ROS_SYSTEM_SLAM_HPP
#define ORB_SLAM3_ROS_SYSTEM_SLAM_HPP

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <ORB_SLAM3/System.h>
#include <sophus/se3.hpp>

#include "orbslam3_ros/pose_snapshot.hpp"

namespace orbslam3_ros {

    enum class SensorMode {
        Monocular,
        Stereo,
        Rgbd,
        StereoInertial,
    };

    class SystemSlam {
    public:
        explicit SystemSlam(
            const std::string& vocab_file,
            const std::string& settings_file,
            SensorMode sensor_mode,
            bool use_viewer = true,
            const std::string& sequence_name = std::string()
        );
        ~SystemSlam();

        SystemSlam(const SystemSlam&) = delete;
        SystemSlam& operator=(const SystemSlam&) = delete;
        SystemSlam(SystemSlam&&) noexcept = default;
        SystemSlam& operator=(SystemSlam&&) noexcept = default;

        [[nodiscard]] bool IsReady() const noexcept;
        [[nodiscard]] SensorMode GetSensorMode() const noexcept;

        void Shutdown();
        void SaveTUMTrajectories(
            const std::filesystem::path& camera_trajectory_file,
            const std::filesystem::path& keyframe_trajectory_file
        );
        void SaveEuRoCTrajectories(
            const std::filesystem::path& camera_trajectory_file,
            const std::filesystem::path& keyframe_trajectory_file
        );

        [[nodiscard]] std::optional<Sophus::SE3f> TrackMonocular(
            const cv::Mat& image,
            double timestamp,
            const std::vector<ORB_SLAM3::IMU::Point>& imu_measurements = {},
            const std::string& filename = std::string()
        );

        [[nodiscard]] std::optional<Sophus::SE3f> TrackStereo(
            const cv::Mat& left_image,
            const cv::Mat& right_image,
            double timestamp,
            const std::vector<ORB_SLAM3::IMU::Point>& imu_measurements = {},
            const std::string& filename = std::string()
        );

        [[nodiscard]] std::optional<Sophus::SE3f> TrackRgbd(
            const cv::Mat& image,
            const cv::Mat& depth_image,
            double timestamp,
            const std::vector<ORB_SLAM3::IMU::Point>& imu_measurements = {},
            const std::string& filename = std::string()
        );

        [[nodiscard]] TrackingState GetTrackingState();
        [[nodiscard]] std::vector<MapPointSnapshot> GetMapPointsSnapshot();

    private:
        [[nodiscard]] static ORB_SLAM3::System::eSensor ToSensorType(SensorMode sensor_mode) noexcept;

        std::unique_ptr<ORB_SLAM3::System> slam_;
        SensorMode sensor_mode_;
    };

}  // namespace orbslam3_ros

#endif  // ORB_SLAM3_ROS_SYSTEM_SLAM_HPP
