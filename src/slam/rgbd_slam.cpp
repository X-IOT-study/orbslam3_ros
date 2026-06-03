/**
 * @file rgbd_slam.cpp
 * @brief RGB-D wrapper around the ORB-SLAM3 system API.
 */

#include "orbslam3_ros/rgbd_slam.hpp"

#include <Eigen/Core>
#include <vector>

namespace orbslam3_ros {
    RGBDSlam::RGBDSlam(const std::string& vocab_file, const std::string& settings_file, bool use_viewer) {
        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocab_file,
            settings_file,
            ORB_SLAM3::System::RGBD,
            use_viewer
        );
    }

    RGBDSlam::~RGBDSlam() {
        Shutdown();
    }

    bool RGBDSlam::IsReady() const noexcept {
        return static_cast<bool>(slam_);
    }

    void RGBDSlam::Shutdown() {
        if (!slam_) {
            return;
        }

        slam_->Shutdown();
        slam_.reset();
    }

    void RGBDSlam::ShutdownAndSaveTUMTrajectories(
        const std::filesystem::path& trajectory_file,
        const std::filesystem::path& keyframe_file
    ) {
        if (!slam_) {
            return;
        }

        slam_->Shutdown();
        slam_->SaveTrajectoryTUM(trajectory_file.string());
        slam_->SaveKeyFrameTrajectoryTUM(keyframe_file.string());
        slam_.reset();
    }

    std::optional<Sophus::SE3f> RGBDSlam::Track(const cv::Mat& rgb, const cv::Mat& depth, double timestamp) {
        if (!IsReady() || rgb.empty() || depth.empty()) {
            return std::nullopt;
        }

        return slam_->TrackRGBD(rgb, depth, timestamp);
    }

    TrackingState RGBDSlam::GetTrackingState() {
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

    std::vector<MapPointSnapshot> RGBDSlam::GetMapPointsSnapshot() {
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
}
