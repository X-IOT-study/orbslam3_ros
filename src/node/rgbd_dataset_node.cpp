/**
 * @file rgbd_dataset_node.cpp
 * @brief Dataset-driven RGB-D ROS node implementation.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#include "orbslam3_ros/rgbd_dataset_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <stdexcept>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace orbslam3_ros {
    RGBDDatasetNode::RGBDDatasetNode() : LocalizationNodeBase("orbslam3_dataset_node") {
        DeclareParameters();
        InitializeSlamAndDataset();
        RCLCPP_INFO(this->get_logger(), "RGBD dataset node created");
    }

    RGBDDatasetNode::~RGBDDatasetNode() {
        FinalizePlayback();
    }

    void RGBDDatasetNode::Start() {
        if (started_) {
            return;
        }

        output_publishers_ = std::make_unique<LocalizationPublishers>(
            shared_from_this(),
            LocalizationPublisherConfig{
                pose_topic_,
                odom_topic_,
                path_topic_,
                tracking_state_topic_,
                map_points_topic_,
                map_frame_,
                base_frame_,
                std::max<std::size_t>(1, queue_size_),
                path_history_size_,
                path_update_distance_,
                path_update_interval_sec_
            }
        );

        InitializeTfInfrastructure();
        PublishStaticTransforms();
        SetupPlayback();
        SetupMapPointsTimer();
        if (!LoadNextPendingFrame()) {
            playback_complete_.store(true);
        }
        started_ = true;
        RCLCPP_INFO(this->get_logger(), "RGBD dataset node started");
    }

    void RGBDDatasetNode::DeclareParameters() {
        auto make_double_descriptor = [](const std::string& description, double from_value, double to_value) {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.description = description;
            descriptor.floating_point_range.resize(1);
            descriptor.floating_point_range[0].from_value = from_value;
            descriptor.floating_point_range[0].to_value = to_value;
            descriptor.floating_point_range[0].step = 0.0;
            return descriptor;
        };

        DeclareCommonParameters();
        association_file_ = this->declare_parameter<std::string>("association_file", "");
        const int queue_size = this->declare_parameter<int>("queue_size", 10);
        queue_size_ = static_cast<std::size_t>(std::max(1, queue_size));
        playback_rate_ = this->declare_parameter<double>("playback_rate", 1.0);
        loop_ = this->declare_parameter<bool>("loop", false);
        map_points_rate_ = this->declare_parameter<double>(
            "map_points_rate",
            30.0,
            make_double_descriptor("Map points publishing frequency in Hz.", 0.1, 60.0)
        );
    }

    void RGBDDatasetNode::InitializeSlamAndDataset() {
        if (!ValidateFilePath(vocab_file_, "vocab_file") ||
            !ValidateFilePath(settings_file_, "settings_file") ||
            !ValidateFilePath(association_file_, "association_file")) {
            throw std::runtime_error("Invalid dataset node parameters");
        }

        output_directory_ = std::filesystem::path(association_file_).parent_path();
        if (output_directory_.empty()) {
            output_directory_ = std::filesystem::current_path();
        }

        RCLCPP_INFO(this->get_logger(), "Initializing dataset RGBD SLAM...");
        slam_ = std::make_unique<RGBDSlam>(vocab_file_, settings_file_, use_viewer_);

        if (!loader_.Load(association_file_)) {
            throw std::runtime_error("Failed to load dataset association file");
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu dataset frames", loader_.Size());
    }

    void RGBDDatasetNode::SetupPlayback() {
        playback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]() {
                OnPlaybackTick();
            }
        );
        StartWorkerThread();
    }

    void RGBDDatasetNode::SetupMapPointsTimer() {
        if (map_points_rate_ <= 0.0) {
            return;
        }

        const auto period = std::chrono::duration<double>(1.0 / map_points_rate_);
        map_points_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() {
                PublishCachedMapPoints();
            }
        );
    }

    void RGBDDatasetNode::FinalizePlayback() {
        if (playback_finished_) {
            return;
        }

        playback_finished_ = true;
        stop_requested_.store(true);

        if (playback_timer_) {
            playback_timer_->cancel();
        }

        if (map_points_timer_) {
            map_points_timer_->cancel();
        }

        worker_cv_.notify_all();
        StopWorkerThread();

        pending_frame_.reset();
        sequence_timing_initialized_ = false;

        if (slam_) {
            const auto camera_trajectory = BuildTrajectoryPath("CameraTrajectory.txt");
            const auto keyframe_trajectory = BuildTrajectoryPath("KeyFrameTrajectory.txt");

            RCLCPP_INFO(
                this->get_logger(),
                "Saving trajectories to %s and %s",
                camera_trajectory.string().c_str(),
                keyframe_trajectory.string().c_str()
            );

            slam_->ShutdownAndSaveTUMTrajectories(camera_trajectory, keyframe_trajectory);
        }

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void RGBDDatasetNode::StartWorkerThread() {
        stop_requested_.store(false);
        worker_busy_.store(false);
        playback_complete_.store(false);
        next_sequence_id_ = 0;
        frame_queue_ = std::make_unique<PlaybackQueue>(std::max<std::size_t>(1, queue_size_));
        worker_thread_ = std::thread(&RGBDDatasetNode::WorkerLoop, this);
    }

    void RGBDDatasetNode::StopWorkerThread() {
        stop_requested_.store(true);
        worker_cv_.notify_all();

        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }

        if (frame_queue_) {
            frame_queue_->Clear();
            frame_queue_.reset();
        }
    }

    bool RGBDDatasetNode::ValidateFilePath(const std::string& path, const char* param_name) {
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter '%s' is empty.", param_name);
            return false;
        }

        const std::filesystem::path file_path(path);
        if (!std::filesystem::exists(file_path) || !std::filesystem::is_regular_file(file_path)) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Parameter '%s' points to a missing or invalid file: %s",
                param_name,
                path.c_str()
            );
            return false;
        }

        return true;
    }

    builtin_interfaces::msg::Time RGBDDatasetNode::ToStamp(double seconds) noexcept {
        const auto total_nanoseconds = static_cast<int64_t>(seconds * 1e9);
        builtin_interfaces::msg::Time stamp;
        stamp.sec = static_cast<int32_t>(total_nanoseconds / 1000000000LL);
        stamp.nanosec = static_cast<uint32_t>(total_nanoseconds % 1000000000LL);
        return stamp;
    }

    cv::Mat RGBDDatasetNode::LoadImage(const std::filesystem::path& path, int flags) {
        return cv::imread(path.string(), flags);
    }

    std::filesystem::path RGBDDatasetNode::BuildTrajectoryPath(const char* filename) const {
        return output_directory_ / filename;
    }

    bool RGBDDatasetNode::LoadNextPendingFrame() {
        TUMDatasetLoader::Frame frame;
        if (loader_.Next(frame)) {
            pending_frame_ = frame;
            if (!sequence_timing_initialized_) {
                sequence_start_stamp_ = frame.timestamp;
                sequence_start_wall_time_ = std::chrono::steady_clock::now();
                sequence_timing_initialized_ = true;
            }
            return true;
        }

        if (!loop_) {
            pending_frame_.reset();
            return false;
        }

        loader_.Reset();
        if (loader_.Next(frame)) {
            pending_frame_ = frame;
            sequence_start_stamp_ = frame.timestamp;
            sequence_start_wall_time_ = std::chrono::steady_clock::now();
            sequence_timing_initialized_ = true;
            return true;
        }

        pending_frame_.reset();
        return false;
    }

    bool RGBDDatasetNode::IsFrameDue() const noexcept {
        if (!pending_frame_) {
            return false;
        }

        const double rate = playback_rate_ > 0.0 ? playback_rate_ : 1.0;
        const double elapsed_seconds = (pending_frame_->timestamp - sequence_start_stamp_) / rate;
        const auto target_time = sequence_start_wall_time_ +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(elapsed_seconds)
            );

        return std::chrono::steady_clock::now() >= target_time;
    }

    void RGBDDatasetNode::OnPlaybackTick() {
        if (playback_finished_ || !slam_) {
            return;
        }

        if (playback_complete_.load() &&
            (!frame_queue_ || frame_queue_->Empty()) &&
            !worker_busy_.load()) {
            RCLCPP_INFO(this->get_logger(), "Dataset playback finished");
            FinalizePlayback();
            return;
        }

        if (!pending_frame_) {
            if (!LoadNextPendingFrame()) {
                playback_complete_.store(true);
            }
            return;
        }

        if (!IsFrameDue()) {
            return;
        }

        const auto frame = *pending_frame_;
        cv::Mat rgb = LoadImage(frame.rgb_path, cv::IMREAD_COLOR);
        cv::Mat depth = LoadImage(frame.depth_path, cv::IMREAD_UNCHANGED);

        if (rgb.empty() || depth.empty()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to load dataset frame: %s / %s",
                frame.rgb_path.string().c_str(),
                frame.depth_path.string().c_str()
            );
            FinalizePlayback();
            return;
        }

        if (!frame_queue_) {
            return;
        }

        auto packet = std::make_shared<PlaybackFrame>();
        packet->sequence_id = next_sequence_id_++;
        packet->stamp = ToStamp(frame.timestamp);
        packet->timestamp = frame.timestamp;
        packet->rgb = std::move(rgb);
        packet->depth = std::move(depth);

        const auto result = frame_queue_->Push(std::move(packet));
        if (result.dropped_oldest) {
            RCLCPP_WARN(
                this->get_logger(),
                "Dropped dataset frame due to queue pressure | sequence=%llu | stamp=%s",
                static_cast<unsigned long long>(packet->sequence_id),
                FormatStamp(ToStamp(frame.timestamp)).c_str()
            );
        }

        worker_cv_.notify_one();

        pending_frame_.reset();
        if (!LoadNextPendingFrame()) {
            playback_complete_.store(true);
        }
    }

    void RGBDDatasetNode::WorkerLoop() {
        while (!stop_requested_.load()) {
            std::shared_ptr<PlaybackFrame> packet;

            if (frame_queue_ && frame_queue_->Pop(packet)) {
                if (packet) {
                    worker_busy_.store(true);
                    ProcessFrame(*packet);
                    worker_busy_.store(false);
                }
                continue;
            }

            std::unique_lock<std::mutex> lock(worker_mutex_);
            worker_cv_.wait_for(lock, std::chrono::milliseconds(5), [this]() {
                return stop_requested_.load() || (frame_queue_ && !frame_queue_->Empty());
            });
        }
    }

    void RGBDDatasetNode::ProcessFrame(const PlaybackFrame& frame) {
        if (!slam_) {
            return;
        }

        const auto pose = slam_->Track(frame.rgb, frame.depth, frame.timestamp);
        const auto tracking_state = slam_->GetTrackingState();
        auto map_points = slam_->GetMapPointsSnapshot();

        PoseSnapshot snapshot;
        snapshot.stamp = frame.stamp;
        snapshot.tracking_state = tracking_state;
        snapshot.map_points = std::move(map_points);
        snapshot.pose_valid = static_cast<bool>(pose);
        if (pose) {
            snapshot = BuildPoseSnapshot(*pose, snapshot.stamp, tracking_state, std::move(snapshot.map_points));
        }

        {
            std::lock_guard<std::mutex> lock(latest_snapshot_mutex_);
            latest_output_snapshot_ = snapshot;
        }

        if (output_publishers_) {
            PoseSnapshot publish_snapshot = snapshot;
            if (pose && invert_pose_) {
                publish_snapshot.pose = ToPoseMessage(*pose);
            }
            output_publishers_->PublishTrackingState(tracking_state);
            if (publish_snapshot.pose_valid) {
                output_publishers_->PublishPoseBundle(publish_snapshot, snapshot);
                output_publishers_->PublishPath(snapshot);
                PublishDynamicTransform(snapshot);
            }
        }
    }

    void RGBDDatasetNode::PublishCachedMapPoints() {
        if (!output_publishers_) {
            return;
        }

        PoseSnapshot snapshot;
        {
            std::lock_guard<std::mutex> lock(latest_snapshot_mutex_);
            snapshot = latest_output_snapshot_;
        }

        output_publishers_->PublishMapPoints(snapshot);
    }
}
