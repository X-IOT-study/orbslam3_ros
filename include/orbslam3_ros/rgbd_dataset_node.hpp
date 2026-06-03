/**
 * @file rgbd_dataset_node.hpp
 * @brief Dataset-driven RGB-D ROS node interface.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_BRIDGE_RGBD_DATASET_NODE_HPP
#define ORB_SLAM3_BRIDGE_RGBD_DATASET_NODE_HPP

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <condition_variable>
#include <atomic>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <mutex>
#include <thread>
#include <builtin_interfaces/msg/time.hpp>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>

#include "orbslam3_ros/node_base.hpp"
#include "orbslam3_ros/publishers.hpp"
#include "orbslam3_ros/tum_dataset_loader.hpp"
#include "orbslam3_ros/rgbd_slam.hpp"
#include "orbslam3_ros/spsc_ring_buffer.hpp"

namespace orbslam3_ros {
    class RGBDDatasetNode : public LocalizationNodeBase {
    public:
        // Construct the dataset playback node.
        RGBDDatasetNode();
        // Stop playback and save the exported trajectories.
        ~RGBDDatasetNode() override;

        RGBDDatasetNode(const RGBDDatasetNode&) = delete;
        RGBDDatasetNode& operator=(const RGBDDatasetNode&) = delete;

        // Start dataset playback, timers, and the SLAM worker thread.
        void Start();

    private:
        struct PlaybackFrame {
            std::uint64_t sequence_id{0};
            builtin_interfaces::msg::Time stamp;
            cv::Mat rgb;
            cv::Mat depth;
            double timestamp{0.0};
        };

        using PlaybackQueue = SpscRingBuffer<PlaybackFrame>;

        void DeclareParameters();
        void InitializeSlamAndDataset();
        void SetupPlayback();
        void SetupMapPointsTimer();
        void StartWorkerThread();
        void StopWorkerThread();
        void FinalizePlayback();
        void OnPlaybackTick();
        void WorkerLoop();
        void ProcessFrame(const PlaybackFrame& frame);
        [[nodiscard]] bool LoadNextPendingFrame();
        [[nodiscard]] bool IsFrameDue() const noexcept;
        [[nodiscard]] bool ValidateFilePath(const std::string& path, const char* param_name);
        [[nodiscard]] static cv::Mat LoadImage(const std::filesystem::path& path, int flags);
        [[nodiscard]] static builtin_interfaces::msg::Time ToStamp(double seconds) noexcept;
        [[nodiscard]] std::filesystem::path BuildTrajectoryPath(const char* filename) const;
        void PublishCachedMapPoints();

        std::string association_file_;
        std::filesystem::path output_directory_;
        std::optional<TUMDatasetLoader::Frame> pending_frame_;
        std::uint64_t next_sequence_id_{0};
        std::chrono::steady_clock::time_point sequence_start_wall_time_{};
        double sequence_start_stamp_{0.0};
        bool sequence_timing_initialized_{false};
        std::size_t queue_size_{10};
        double playback_rate_{1.0};
        bool loop_{false};
        double map_points_rate_{30.0};

        bool started_{false};
        bool playback_finished_{false};
        std::atomic<bool> stop_requested_{false};
        std::atomic<bool> worker_busy_{false};
        std::atomic<bool> playback_complete_{false};
        rclcpp::TimerBase::SharedPtr playback_timer_;
        rclcpp::TimerBase::SharedPtr map_points_timer_;
        std::thread worker_thread_;
        std::condition_variable worker_cv_;
        std::mutex worker_mutex_;
        std::unique_ptr<PlaybackQueue> frame_queue_;
        std::unique_ptr<LocalizationPublishers> output_publishers_;
        std::mutex latest_snapshot_mutex_;
        PoseSnapshot latest_output_snapshot_;
        TUMDatasetLoader loader_;
        std::unique_ptr<RGBDSlam> slam_;
    };
}

#endif  // ORB_SLAM3_BRIDGE_RGBD_DATASET_NODE_HPP
