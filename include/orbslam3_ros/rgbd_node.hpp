/**
 * @file rgbd_node.hpp
 * @brief RGB-D ROS node interface.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_BRIDGE_RGBD_NODE_HPP
#define ORB_SLAM3_BRIDGE_RGBD_NODE_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <builtin_interfaces/msg/time.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "orbslam3_ros/node_base.hpp"
#include "orbslam3_ros/publishers.hpp"
#include "orbslam3_ros/rgbd_slam.hpp"
#include "orbslam3_ros/spsc_ring_buffer.hpp"

namespace orbslam3_ros {

    using ApproximateSyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using ImageSynchronizer = message_filters::Synchronizer<ApproximateSyncPolicy>;

    class RGBDNode : public LocalizationNodeBase {
    public:
        // Construct the realtime RGB-D SLAM node.
        RGBDNode();
        // Shut down the worker thread and SLAM backend.
        ~RGBDNode() override;

        RGBDNode(const RGBDNode&) = delete;
        RGBDNode& operator=(const RGBDNode&) = delete;

        // Start subscriptions, timers, and the SLAM worker thread.
        void Start();

    private:
        struct SyncedFrame {
            std::uint64_t sequence_id{0};
            builtin_interfaces::msg::Time rgb_stamp;
            builtin_interfaces::msg::Time depth_stamp;
            sensor_msgs::msg::Image::ConstSharedPtr rgb_msg;
            sensor_msgs::msg::Image::ConstSharedPtr depth_msg;
            double sync_delta_sec{0.0};
        };

        using FrameQueue = SpscRingBuffer<SyncedFrame>;

        void DeclareParameters();
        void SetupSubscriptions();
        void SetupDiagnostics();
        void SetupMapPointsTimer();
        void StartWorkerThread();
        void StopWorkerThread();
        void OnSyncedFrames(
            const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg
        );
        void OnCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
        void WorkerLoop();
        void HandleFrame(const SyncedFrame& frame);
        void TryInitializeSlam(
            const SyncedFrame& seed_frame,
            const sensor_msgs::msg::CameraInfo& camera_info
        );
        void ProcessFrame(const SyncedFrame& frame);
        [[nodiscard]] bool BuildRuntimeSettingsFile(
            const sensor_msgs::msg::CameraInfo& camera_info,
            const sensor_msgs::msg::Image& rgb_image,
            std::filesystem::path& runtime_settings_file
        );
        [[nodiscard]] bool ValidateCameraInfo(
            const sensor_msgs::msg::CameraInfo& camera_info,
            const sensor_msgs::msg::Image& rgb_image
        ) const;
        [[nodiscard]] bool IsMonotonic(const SyncedFrame& frame) const noexcept;
        void LogDiagnostics();
        void PublishCachedMapPoints();
        [[nodiscard]] static bool IsRgbEncoding(const std::string& encoding) noexcept;
        [[nodiscard]] static std::string FormatDouble(double value);

        std::string rgb_topic_{"rgb/image_raw"};
        std::string depth_topic_{"depth/image_raw"};
        std::string rgb_camera_info_topic_{"/camera/color/camera_info"};
        double stats_hz_{1.0};
        double map_points_rate_{30.0};
        double sync_tolerance_sec_{0.05};
        bool diagnostic_logging_{false};
        std::size_t queue_size_{10};       // Legacy compatibility alias.
        std::size_t sync_queue_size_{10};
        std::size_t frame_queue_size_{10};

        bool started_{false};
        std::atomic<bool> stop_requested_{false};
        std::atomic<bool> slam_initialized_{false};
        std::atomic<bool> initialization_retry_pending_{false};
        std::atomic<bool> camera_info_wait_logged_{false};
        std::atomic<bool> camera_info_ready_logged_{false};
        std::atomic<std::uint8_t> last_tracking_state_code_{255};
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
        image_transport::SubscriberFilter rgb_subscription_;
        image_transport::SubscriberFilter depth_subscription_;
        std::shared_ptr<ImageSynchronizer> synchronizer_;
        std::unique_ptr<FrameQueue> frame_queue_;
        std::thread worker_thread_;
        std::condition_variable worker_cv_;
        std::mutex worker_mutex_;
        std::mutex camera_info_mutex_;
        std::optional<sensor_msgs::msg::CameraInfo> latest_camera_info_;
        std::optional<SyncedFrame> pending_initial_frame_;
        std::filesystem::path runtime_settings_path_;
        rclcpp::TimerBase::SharedPtr diagnostics_timer_;
        std::atomic<std::uint64_t> frame_sequence_{0};
        std::uint64_t last_processed_sequence_{0};
        std::int64_t last_processed_rgb_stamp_ns_{-1};
        std::atomic<std::uint64_t> synced_pairs_total_{0};
        std::atomic<std::uint64_t> processed_frames_total_{0};
        std::atomic<std::uint64_t> dropped_sync_total_{0};
        std::atomic<std::uint64_t> dropped_queue_total_{0};
        std::atomic<std::uint64_t> dropped_duplicate_total_{0};
        std::atomic<std::uint64_t> monotonic_sequence_drop_total_{0};
        std::atomic<std::uint64_t> monotonic_stamp_drop_total_{0};
        std::atomic<std::uint64_t> initialization_attempt_total_{0};
        std::atomic<std::uint64_t> initialization_failure_total_{0};
        std::atomic<std::int64_t> sync_delay_sum_ns_{0};
        std::atomic<std::int64_t> latest_rgb_stamp_ns_{-1};
        std::atomic<std::int64_t> latest_depth_stamp_ns_{-1};
        std::atomic<std::int64_t> latest_sync_delta_ns_{-1};
        std::atomic<std::int64_t> max_sync_delta_ns_{0};
        std::atomic<std::uint64_t> camera_info_total_{0};
        std::atomic<std::int64_t> latest_camera_info_stamp_ns_{-1};
        std::uint64_t diagnostics_last_processed_total_{0};
        std::int64_t diagnostics_last_sync_delay_sum_ns_{0};
        std::chrono::steady_clock::time_point diagnostics_last_report_time_{};
        rclcpp::TimerBase::SharedPtr map_points_timer_;
        std::unique_ptr<LocalizationPublishers> output_publishers_;
        std::mutex latest_snapshot_mutex_;
        PoseSnapshot latest_output_snapshot_;
        std::unique_ptr<RGBDSlam> slam_;    // Unique pointer to the RGBD SLAM wrapper.
    };
}

#endif  // ORB_SLAM3_BRIDGE_RGBD_NODE_HPP
