/**
 * @file multimode_node.hpp
 * @brief Multi-mode ORB-SLAM3 ROS node interface.
 * @author WenSheng Xu
 * @date 2026-06-04
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#pragma once
#ifndef ORB_SLAM3_ROS_MULTIMODE_NODE_HPP
#define ORB_SLAM3_ROS_MULTIMODE_NODE_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "orbslam3_ros/node_base.hpp"
#include "orbslam3_ros/publishers.hpp"
#include "orbslam3_ros/spsc_ring_buffer.hpp"
#include "orbslam3_ros/slam/system_slam.hpp"

namespace orbslam3_ros {

    using ImagePairSyncPolicy =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using ImagePairSynchronizer = message_filters::Synchronizer<ImagePairSyncPolicy>;

    class MultiModeNode : public LocalizationNodeBase {
    public:
        explicit MultiModeNode();
        ~MultiModeNode() override;

        MultiModeNode(const MultiModeNode&) = delete;
        MultiModeNode& operator=(const MultiModeNode&) = delete;

        void Start();

    private:
        enum class RunMode {
            Realtime,
            Dataset,
        };

        struct FramePacket {
            std::uint64_t sequence_id{0};
            builtin_interfaces::msg::Time stamp;
            cv::Mat primary_image;
            cv::Mat secondary_image;
            cv::Mat depth_image;
            std::vector<ORB_SLAM3::IMU::Point> imu_measurements;
            std::string filename;
            std::string encoding;
            int width{0};
            int height{0};
        };

        struct DatasetEntry {
            double timestamp{0.0};
            std::filesystem::path primary_path;
            std::filesystem::path secondary_path;
            std::filesystem::path depth_path;
            std::string filename;
        };

        struct ImuEntry {
            double timestamp{0.0};
            ORB_SLAM3::IMU::Point measurement;
        };

        using FrameQueue = SpscRingBuffer<FramePacket>;

        void DeclareParameters();
        void InitializeOutputs();
        void InitializeRuntimeSlam();
        void StartRealtime();
        void StartDatasetPlayback();
        void SetupMonoSubscriptions();
        void SetupStereoSubscriptions();
        void SetupRgbdSubscriptions();
        void SetupStereoInertialSubscriptions();
        void SetupCommonTimers();
        void StartWorkerThread();
        void StopWorkerThread();
        void WorkerLoop();
        void HandleFrame(const FramePacket& packet);
        void ProcessFrame(const FramePacket& packet);
        void PublishCachedMapPoints();
        void LogDiagnostics();
        void OnMonoImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        void OnStereoImages(
            const sensor_msgs::msg::Image::ConstSharedPtr& left_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& right_msg
        );
        void OnRgbdImages(
            const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg
        );
        void OnRgbdCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg);
        void OnImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg);
        void OnPlaybackTick();
        [[nodiscard]] bool LoadDatasetFrames();
        [[nodiscard]] bool LoadNextDatasetFrame();
        [[nodiscard]] bool IsDatasetFrameDue() const noexcept;
        [[nodiscard]] bool IsMonotonic(const FramePacket& packet) const noexcept;
        [[nodiscard]] bool IsRgbEncoding(const std::string& encoding) const noexcept;
        [[nodiscard]] static std::string ToLowerCopy(std::string value);
        [[nodiscard]] static std::string TrimCopy(const std::string& value);
        [[nodiscard]] static bool IsIgnorableLine(const std::string& line);
        [[nodiscard]] static std::filesystem::path ResolvePath(
            const std::filesystem::path& base_path,
            const std::string& file_path
        );
        [[nodiscard]] static builtin_interfaces::msg::Time ToStamp(double seconds) noexcept;
        [[nodiscard]] static double ToSeconds(const builtin_interfaces::msg::Time& stamp) noexcept;
        [[nodiscard]] static std::int64_t ToNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept;
        [[nodiscard]] static double ToMilliseconds(std::int64_t nanoseconds) noexcept;
        [[nodiscard]] static std::string FormatStamp(const builtin_interfaces::msg::Time& stamp);
        [[nodiscard]] static std::string FormatStampNs(std::int64_t nanoseconds);
        [[nodiscard]] static std::string FormatDouble(double value);
        [[nodiscard]] static std::string NormalizeModeName(std::string value);
        [[nodiscard]] static SensorMode ParseSensorMode(const std::string& mode);
        [[nodiscard]] static RunMode ParseRunMode(const std::string& mode);
        [[nodiscard]] static bool RewriteSettingsFile(
            const std::filesystem::path& source_path,
            const std::filesystem::path& destination_path,
            const std::vector<std::pair<std::string, std::string>>& overrides
        );
        [[nodiscard]] bool ValidateCameraInfo(
            const sensor_msgs::msg::CameraInfo& camera_info,
            const FramePacket& packet
        ) const;
        [[nodiscard]] bool BuildRuntimeSettingsFile(
            const sensor_msgs::msg::CameraInfo& camera_info,
            const FramePacket& packet,
            std::filesystem::path& runtime_settings_file
        );
        [[nodiscard]] std::vector<ORB_SLAM3::IMU::Point> CollectImuMeasurements(double timestamp);
        [[nodiscard]] std::filesystem::path BuildTrajectoryPath(const char* filename) const;
        void FinalizeDatasetPlayback();

        SensorMode input_mode_{SensorMode::Rgbd};
        RunMode run_mode_{RunMode::Realtime};
        bool runtime_calibration_from_camera_info_{false};
        bool started_{false};

        std::string mode_name_{"rgbd"};
        std::string run_mode_name_{"realtime"};
        std::string dataset_root_;
        std::string image_topic_{"image_raw"};
        std::string left_topic_{"left/image_raw"};
        std::string right_topic_{"right/image_raw"};
        std::string rgb_topic_{"rgb/image_raw"};
        std::string depth_topic_{"depth/image_raw"};
        std::string imu_topic_{"imu"};
        std::string image_transport_{"raw"};
        std::string left_transport_{"raw"};
        std::string right_transport_{"raw"};
        std::string rgb_transport_{"raw"};
        std::string depth_transport_{"raw"};
        std::string rgb_camera_info_topic_{"camera_info"};
        std::string association_file_;
        std::string imu_file_;
        double stats_hz_{1.0};
        double map_points_rate_{30.0};
        double sync_tolerance_sec_{0.05};
        double playback_rate_{1.0};
        bool diagnostic_logging_{false};
        bool loop_{false};
        std::size_t queue_size_{10};
        std::size_t sync_queue_size_{10};
        std::size_t frame_queue_size_{10};

        std::unique_ptr<SystemSlam> slam_;
        std::unique_ptr<LocalizationPublishers> output_publishers_;
        std::unique_ptr<FrameQueue> frame_queue_;
        rclcpp::TimerBase::SharedPtr diagnostics_timer_;
        rclcpp::TimerBase::SharedPtr playback_timer_;
        rclcpp::TimerBase::SharedPtr map_points_timer_;
        std::thread worker_thread_;
        std::condition_variable worker_cv_;
        std::mutex worker_mutex_;
        std::atomic<bool> stop_requested_{false};
        std::atomic<bool> worker_busy_{false};
        std::atomic<bool> slam_initialized_{false};
        std::atomic<bool> initialization_retry_pending_{false};
        std::atomic<bool> playback_complete_{false};
        std::atomic<std::uint64_t> frame_sequence_{0};
        std::uint64_t last_processed_sequence_{0};
        std::int64_t last_processed_stamp_ns_{-1};
        std::atomic<std::uint64_t> processed_frames_total_{0};
        std::atomic<std::uint64_t> dropped_queue_total_{0};
        std::atomic<std::uint64_t> dropped_duplicate_total_{0};
        std::atomic<std::uint64_t> sync_pair_total_{0};
        std::atomic<std::int64_t> sync_delay_sum_ns_{0};
        std::atomic<std::uint64_t> imu_messages_total_{0};
        std::atomic<std::uint64_t> camera_info_total_{0};
        std::atomic<std::int64_t> latest_camera_info_stamp_ns_{-1};
        std::atomic<std::int64_t> latest_sync_delta_ns_{-1};
        std::atomic<std::int64_t> max_sync_delta_ns_{0};
        std::chrono::steady_clock::time_point diagnostics_last_report_time_{};
        std::uint64_t diagnostics_last_processed_total_{0};

        image_transport::SubscriberFilter mono_subscription_;
        image_transport::SubscriberFilter left_subscription_;
        image_transport::SubscriberFilter right_subscription_;
        image_transport::SubscriberFilter rgb_subscription_;
        image_transport::SubscriberFilter depth_subscription_;
        std::shared_ptr<ImagePairSynchronizer> stereo_sync_;
        std::shared_ptr<ImagePairSynchronizer> rgbd_sync_;
        std::shared_ptr<ImagePairSynchronizer> stereo_inertial_sync_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
        std::mutex imu_mutex_;
        std::mutex camera_info_mutex_;
        std::mutex latest_snapshot_mutex_;
        std::optional<sensor_msgs::msg::CameraInfo> latest_camera_info_;
        PoseSnapshot latest_output_snapshot_;

        std::vector<DatasetEntry> dataset_entries_;
        std::vector<ImuEntry> dataset_imu_entries_;
        std::size_t dataset_index_{0};
        std::size_t dataset_imu_index_{0};
        std::optional<FramePacket> pending_initial_frame_;
        std::filesystem::path runtime_settings_path_;
        std::filesystem::path dataset_output_directory_;
        std::chrono::steady_clock::time_point dataset_sequence_start_wall_time_{};
        double dataset_sequence_start_stamp_{0.0};
        bool dataset_timing_initialized_{false};
    };

}  // namespace orbslam3_ros

#endif  // ORB_SLAM3_ROS_MULTIMODE_NODE_HPP
