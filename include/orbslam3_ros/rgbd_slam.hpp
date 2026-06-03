/**
 * @file rgbd_slam.hpp
 * @brief RGB-D SLAM wrapper interface.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_BRIDGE_RGBD_SLAM_HPP
#define ORB_SLAM3_BRIDGE_RGBD_SLAM_HPP

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
    class RGBDSlam {
    public:
        // Construct the ORB-SLAM3 RGB-D wrapper.
        explicit RGBDSlam(const std::string& vocab_file, const std::string& settings_file, bool use_viewer = true);
        // Shut down the SLAM system.
        ~RGBDSlam();

        RGBDSlam(const RGBDSlam&) = delete;
        RGBDSlam& operator=(const RGBDSlam&) = delete;
        RGBDSlam(RGBDSlam&&) noexcept = default;
        RGBDSlam& operator=(RGBDSlam&&) noexcept = default;

        // Return true when the wrapper still owns a live SLAM system.
        [[nodiscard]] bool IsReady() const noexcept;
        // Stop the SLAM system and release its resources.
        void Shutdown();
        // Stop the SLAM system and save TUM trajectories to disk.
        void ShutdownAndSaveTUMTrajectories(
            const std::filesystem::path& trajectory_file,
            const std::filesystem::path& keyframe_file
        );

        // Process a single RGB-D frame and return the tracked camera pose.
        // The depth map should already be aligned with the RGB image.
        // Returns std::nullopt when tracking is unavailable.
        [[nodiscard]] std::optional<Sophus::SE3f> Track(const cv::Mat& rgb, const cv::Mat& depth, double timestamp);
        // Return the current ORB-SLAM3 tracking state.
        [[nodiscard]] TrackingState GetTrackingState();
        // Return a snapshot of the currently tracked map points.
        [[nodiscard]] std::vector<MapPointSnapshot> GetMapPointsSnapshot();

    private:
        std::unique_ptr<ORB_SLAM3::System> slam_;
    };
}

#endif  // ORB_SLAM3_BRIDGE_RGBD_SLAM_HPP
