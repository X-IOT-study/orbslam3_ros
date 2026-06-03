/**
 * @file rgbd_node.cpp
 * @brief Realtime RGB-D ROS node implementation.
 * @author WenSheng Xu
 * @date 2026-06-03
 * @version 0.1
 * @copyright Copyright (c) 2026, WenSheng Xu. All rights reserved.
 */

#include "orbslam3_ros/rgbd_node.hpp"

#include <algorithm>
#include <cstdlib>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>

namespace {
    std::string ToLowerCopy(std::string value) {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        return value;
    }

    std::string TrimLeftCopy(const std::string& value) {
        const auto first = value.find_first_not_of(" \t");
        if (first == std::string::npos) {
            return "";
        }
        return value.substr(first);
    }

    bool LineStartsWithKey(const std::string& line, const std::string& key) {
        if (line.size() < key.size()) {
            return false;
        }

        if (line.compare(0, key.size(), key) != 0) {
            return false;
        }

        if (line.size() == key.size()) {
            return true;
        }

        const char next = line[key.size()];
        return next == ':' || std::isspace(static_cast<unsigned char>(next));
    }

    bool RewriteSettingsFile(
        const std::filesystem::path& source_path,
        const std::filesystem::path& destination_path,
        const std::vector<std::pair<std::string, std::string>>& overrides
    ) {
        std::ifstream input(source_path);
        if (!input.is_open()) {
            return false;
        }

        std::ofstream output(destination_path);
        if (!output.is_open()) {
            return false;
        }

        std::set<std::string> applied_keys;
        std::string line;

        while (std::getline(input, line)) {
            const std::string trimmed = TrimLeftCopy(line);
            bool replaced = false;

            for (const auto& [key, value] : overrides) {
                if (LineStartsWithKey(trimmed, key)) {
                    output << key << ": " << value << '\n';
                    applied_keys.insert(key);
                    replaced = true;
                    break;
                }
            }

            if (!replaced) {
                output << line << '\n';
            }
        }

        output << '\n' << "# Runtime RGB-D overrides" << '\n';
        for (const auto& [key, value] : overrides) {
            if (!applied_keys.count(key)) {
                output << key << ": " << value << '\n';
            }
        }

        return output.good();
    }
}

namespace orbslam3_ros {

    RGBDNode::RGBDNode() : LocalizationNodeBase("orbslam3_realtime_node") {
        DeclareParameters();
        RCLCPP_INFO(this->get_logger(), "RGBD realtime node created");
    }

    RGBDNode::~RGBDNode() {
        StopWorkerThread();
    }

    void RGBDNode::Start() {
        if (started_) {
            return;
        }

        if (!ValidateFilePath(vocab_file_, "vocab_file") ||
            !ValidateFilePath(settings_file_, "settings_file")) {
            throw std::runtime_error("Invalid SLAM file parameters");
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

        frame_queue_ = std::make_unique<FrameQueue>(std::max<std::size_t>(1, frame_queue_size_));
        InitializeTfInfrastructure();
        PublishStaticTransforms();

        SetupSubscriptions();
        SetupDiagnostics();
        SetupMapPointsTimer();
        StartWorkerThread();

        diagnostics_last_report_time_ = std::chrono::steady_clock::now();
        started_ = true;
        RCLCPP_INFO(this->get_logger(), "RGBD realtime node started");
    }

    void RGBDNode::DeclareParameters() {
        auto make_integer_descriptor = [](const std::string& description, int from_value, int to_value) {
            rcl_interfaces::msg::ParameterDescriptor descriptor;
            descriptor.description = description;
            descriptor.integer_range.resize(1);
            descriptor.integer_range[0].from_value = from_value;
            descriptor.integer_range[0].to_value = to_value;
            descriptor.integer_range[0].step = 1;
            return descriptor;
        };

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

        rgb_topic_ = this->declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
        depth_topic_ = this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
        this->declare_parameter<std::string>("image_transport", "raw");
        this->declare_parameter<std::string>("depth_transport", "raw");
        rgb_camera_info_topic_ = this->declare_parameter<std::string>("rgb_camera_info_topic", "/camera/color/camera_info");
        const int legacy_queue_size = this->declare_parameter<int>(
            "queue_size",
            10,
            make_integer_descriptor("Legacy compatibility queue size.", 1, 1000)
        );
        queue_size_ = static_cast<std::size_t>(std::max(1, legacy_queue_size));

        const int sync_queue_size = this->declare_parameter<int>(
            "sync_queue_size",
            legacy_queue_size,
            make_integer_descriptor("RGB-D message_filters queue depth.", 1, 1000)
        );
        sync_queue_size_ = static_cast<std::size_t>(std::max(1, sync_queue_size));

        const int frame_queue_size = this->declare_parameter<int>(
            "frame_queue_size",
            legacy_queue_size,
            make_integer_descriptor("Bounded SPSC frame queue depth.", 1, 1000)
        );
        frame_queue_size_ = static_cast<std::size_t>(std::max(1, frame_queue_size));

        sync_tolerance_sec_ = this->declare_parameter<double>(
            "sync_tolerance_sec",
            0.05,
            make_double_descriptor("Maximum RGB/depth timestamp delta in seconds.", 0.03, 0.08)
        );
        stats_hz_ = this->declare_parameter<double>(
            "stats_hz",
            1.0,
            make_double_descriptor("Diagnostic logging frequency in Hz.", 0.1, 10.0)
        );
        map_points_rate_ = this->declare_parameter<double>(
            "map_points_rate",
            30.0,
            make_double_descriptor("Map points publishing frequency in Hz.", 0.1, 60.0)
        );
        diagnostic_logging_ = this->declare_parameter<bool>("diagnostic_logging", false);
    }

    void RGBDNode::SetupSubscriptions() {
        image_transport::TransportHints rgb_hints(this);
        image_transport::TransportHints depth_hints(this, "raw", "depth_transport");

        RCLCPP_INFO(
            this->get_logger(),
            "Image transports | rgb=%s | depth=%s",
            rgb_hints.getTransport().c_str(),
            depth_hints.getTransport().c_str()
        );

        rgb_subscription_.subscribe(
            this,
            rgb_topic_,
            rgb_hints.getTransport(),
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );
        depth_subscription_.subscribe(
            this,
            depth_topic_,
            depth_hints.getTransport(),
            rclcpp::SensorDataQoS().get_rmw_qos_profile()
        );

        synchronizer_ = std::make_shared<ImageSynchronizer>(
            ApproximateSyncPolicy(static_cast<uint32_t>(sync_queue_size_)),
            rgb_subscription_,
            depth_subscription_
        );
        synchronizer_->registerCallback(&RGBDNode::OnSyncedFrames, this);

        camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            rgb_camera_info_topic_,
            rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
                OnCameraInfo(msg);
            }
        );
    }

    void RGBDNode::SetupDiagnostics() {
        if (stats_hz_ <= 0.0) {
            return;
        }

        const auto period = std::chrono::duration<double>(1.0 / stats_hz_);
        diagnostics_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() {
                LogDiagnostics();
            }
        );
    }

    void RGBDNode::SetupMapPointsTimer() {
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

    void RGBDNode::StartWorkerThread() {
        stop_requested_.store(false);
        worker_thread_ = std::thread(&RGBDNode::WorkerLoop, this);
    }

    void RGBDNode::StopWorkerThread() {
        stop_requested_.store(true);
        worker_cv_.notify_all();

        if (diagnostics_timer_) {
            diagnostics_timer_->cancel();
            diagnostics_timer_.reset();
        }

        if (map_points_timer_) {
            map_points_timer_->cancel();
            map_points_timer_.reset();
        }

        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }

        if (frame_queue_) {
            frame_queue_->Clear();
        }

        slam_.reset();
        slam_initialized_.store(false);
        initialization_retry_pending_.store(false);
        latest_camera_info_.reset();
        pending_initial_frame_.reset();
        frame_queue_.reset();
        output_publishers_.reset();

        if (!runtime_settings_path_.empty()) {
            std::error_code ec;
            std::filesystem::remove(runtime_settings_path_, ec);
            runtime_settings_path_.clear();
        }
    }

    void RGBDNode::OnSyncedFrames(
        const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg
    ) {
        if (!frame_queue_) {
            return;
        }

        const std::int64_t rgb_stamp_ns = ToNanoseconds(rgb_msg->header.stamp);
        const std::int64_t depth_stamp_ns = ToNanoseconds(depth_msg->header.stamp);
        const std::int64_t delta_ns = std::llabs(rgb_stamp_ns - depth_stamp_ns);
        const double delta_sec = static_cast<double>(delta_ns) * 1e-9;

        synced_pairs_total_.fetch_add(1);
        sync_delay_sum_ns_.fetch_add(delta_ns);
        latest_rgb_stamp_ns_.store(rgb_stamp_ns);
        latest_depth_stamp_ns_.store(depth_stamp_ns);
        latest_sync_delta_ns_.store(delta_ns);

        std::int64_t observed_max_delta_ns = max_sync_delta_ns_.load();
        while (delta_ns > observed_max_delta_ns &&
               !max_sync_delta_ns_.compare_exchange_weak(
                   observed_max_delta_ns,
                   delta_ns,
                   std::memory_order_relaxed,
                   std::memory_order_relaxed
               )) {
        }

        if (delta_sec > sync_tolerance_sec_) {
            const auto dropped_total = dropped_sync_total_.fetch_add(1) + 1;
            if (diagnostic_logging_ && (dropped_total <= 5 || dropped_total % 20 == 0)) {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Dropped RGB-D pair | rgb=%s | depth=%s | delta=%.3f ms | tolerance=%.3f ms | dropped=%llu",
                    FormatStamp(rgb_msg->header.stamp).c_str(),
                    FormatStamp(depth_msg->header.stamp).c_str(),
                    ToMilliseconds(delta_ns),
                    sync_tolerance_sec_ * 1e3,
                    static_cast<unsigned long long>(dropped_total)
                );
            }
            return;
        }

        SyncedFrame frame;
        frame.sequence_id = frame_sequence_.fetch_add(1) + 1;
        frame.rgb_stamp = rgb_msg->header.stamp;
        frame.depth_stamp = depth_msg->header.stamp;
        frame.rgb_msg = rgb_msg;
        frame.depth_msg = depth_msg;
        frame.sync_delta_sec = delta_sec;

        if (!slam_initialized_.load()) {
            initialization_retry_pending_.store(true);
        }

        const auto result = frame_queue_->Push(std::make_shared<SyncedFrame>(std::move(frame)));
        if (result.dropped_oldest) {
            dropped_queue_total_.fetch_add(1);
        }

        worker_cv_.notify_one();
    }

    void RGBDNode::OnCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
        if (slam_initialized_.load()) {
            return;
        }

        const std::int64_t camera_info_stamp_ns = ToNanoseconds(msg->header.stamp);
        {
            std::lock_guard<std::mutex> lock(camera_info_mutex_);
            latest_camera_info_ = *msg;
        }

        latest_camera_info_stamp_ns_.store(camera_info_stamp_ns);
        camera_info_total_.fetch_add(1);
        if (diagnostic_logging_ && camera_info_total_.load() <= 3) {
            RCLCPP_INFO(
                this->get_logger(),
                "CameraInfo received | stamp=%s | frame_id=%s | size=%ux%u",
                FormatStamp(msg->header.stamp).c_str(),
                msg->header.frame_id.c_str(),
                msg->width,
                msg->height
            );
        }

        initialization_retry_pending_.store(true);
        worker_cv_.notify_one();
    }

    void RGBDNode::WorkerLoop() {
        while (!stop_requested_.load()) {
            std::shared_ptr<SyncedFrame> packet;

            if (frame_queue_ && frame_queue_->Pop(packet)) {
                if (packet) {
                    HandleFrame(*packet);
                }
                continue;
            }

            if (!slam_initialized_.load() && initialization_retry_pending_.exchange(false)) {
                if (pending_initial_frame_) {
                    HandleFrame(*pending_initial_frame_);
                    continue;
                }
            }

            std::unique_lock<std::mutex> lock(worker_mutex_);
            worker_cv_.wait_for(lock, std::chrono::milliseconds(5), [this]() {
                return stop_requested_.load() ||
                       (frame_queue_ && !frame_queue_->Empty()) ||
                       (!slam_initialized_.load() && initialization_retry_pending_.load());
            });
        }
    }

    void RGBDNode::HandleFrame(const SyncedFrame& frame) {
        if (!slam_initialized_.load()) {
            pending_initial_frame_ = frame;
            std::optional<sensor_msgs::msg::CameraInfo> camera_info_snapshot;
            {
                std::lock_guard<std::mutex> lock(camera_info_mutex_);
                if (latest_camera_info_) {
                    camera_info_snapshot = latest_camera_info_;
                }
            }

            if (!camera_info_snapshot) {
                return;
            }

            TryInitializeSlam(frame, *camera_info_snapshot);
            if (!slam_initialized_.load()) {
                return;
            }

            const SyncedFrame seed_frame = *pending_initial_frame_;
            pending_initial_frame_.reset();
            ProcessFrame(seed_frame);
            return;
        }

        ProcessFrame(frame);
    }

    void RGBDNode::TryInitializeSlam(
        const SyncedFrame& seed_frame,
        const sensor_msgs::msg::CameraInfo& camera_info
    ) {
        if (slam_initialized_.load()) {
            return;
        }

        initialization_attempt_total_.fetch_add(1);

        if (!ValidateCameraInfo(camera_info, *seed_frame.rgb_msg)) {
            initialization_failure_total_.fetch_add(1);
            initialization_retry_pending_.store(false);
            return;
        }

        std::filesystem::path runtime_settings_path;
        if (!BuildRuntimeSettingsFile(camera_info, *seed_frame.rgb_msg, runtime_settings_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to build runtime ORB-SLAM3 settings file");
            initialization_failure_total_.fetch_add(1);
            initialization_retry_pending_.store(false);
            return;
        }

        try {
            slam_ = std::make_unique<RGBDSlam>(vocab_file_, runtime_settings_path.string(), use_viewer_);
            runtime_settings_path_ = std::move(runtime_settings_path);
            slam_initialized_.store(true);
            initialization_retry_pending_.store(false);

            RCLCPP_INFO(
                this->get_logger(),
                "ORB-SLAM3 initialized from RGB camera info (%ux%u, fx=%.3f, fy=%.3f, cx=%.3f, cy=%.3f)",
                camera_info.width,
                camera_info.height,
                camera_info.k[0],
                camera_info.k[4],
                camera_info.k[2],
                camera_info.k[5]
            );
        } catch (const std::exception& e) {
            initialization_failure_total_.fetch_add(1);
            initialization_retry_pending_.store(false);
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize ORB-SLAM3: %s", e.what());
        }
    }

    void RGBDNode::ProcessFrame(const SyncedFrame& frame) {
        if (!slam_ || !slam_initialized_.load()) {
            return;
        }

        const std::int64_t rgb_stamp_ns = ToNanoseconds(frame.rgb_stamp);
        if (frame.sequence_id <= last_processed_sequence_) {
            dropped_duplicate_total_.fetch_add(1);
            monotonic_sequence_drop_total_.fetch_add(1);
            RCLCPP_WARN(
                this->get_logger(),
                "Dropped non-monotonic frame sequence | current=%llu | last=%llu | rgb=%s",
                static_cast<unsigned long long>(frame.sequence_id),
                static_cast<unsigned long long>(last_processed_sequence_),
                FormatStamp(frame.rgb_stamp).c_str()
            );
            return;
        }

        if (rgb_stamp_ns <= last_processed_rgb_stamp_ns_) {
            dropped_duplicate_total_.fetch_add(1);
            monotonic_stamp_drop_total_.fetch_add(1);
            RCLCPP_WARN(
                this->get_logger(),
                "Dropped non-monotonic RGB stamp | current=%s | last_ns=%lld",
                FormatStamp(frame.rgb_stamp).c_str(),
                static_cast<long long>(last_processed_rgb_stamp_ns_)
            );
            return;
        }

        last_processed_sequence_ = frame.sequence_id;
        last_processed_rgb_stamp_ns_ = rgb_stamp_ns;
        processed_frames_total_.fetch_add(1);

        try {
            const auto rgb_cv = cv_bridge::toCvShare(frame.rgb_msg, frame.rgb_msg->encoding);
            const auto depth_cv = cv_bridge::toCvShare(frame.depth_msg, frame.depth_msg->encoding);
            const auto pose = slam_->Track(rgb_cv->image, depth_cv->image, ToSeconds(frame.rgb_stamp));
            const auto tracking_state = slam_->GetTrackingState();
            auto map_points = slam_->GetMapPointsSnapshot();

            PoseSnapshot snapshot;
            snapshot.stamp = frame.rgb_stamp;
            snapshot.tracking_state = tracking_state;
            snapshot.map_points = std::move(map_points);
            snapshot.pose_valid = static_cast<bool>(pose);
            if (pose) {
                snapshot = BuildPoseSnapshot(*pose, frame.rgb_stamp, tracking_state, std::move(snapshot.map_points));
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
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert RGB-D frames: %s", e.what());
        }
    }

    bool RGBDNode::BuildRuntimeSettingsFile(
        const sensor_msgs::msg::CameraInfo& camera_info,
        const sensor_msgs::msg::Image& rgb_image,
        std::filesystem::path& runtime_settings_file
    ) {
        std::error_code temp_dir_error;
        const std::filesystem::path temp_dir = std::filesystem::temp_directory_path(temp_dir_error);
        if (temp_dir_error) {
            return false;
        }

        const auto unique_name = std::string("orbslam3_ros_runtime_") +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml";
        runtime_settings_file = temp_dir / unique_name;

        const bool rgb_is_rgb = IsRgbEncoding(rgb_image.encoding);
        const std::vector<std::pair<std::string, std::string>> overrides = {
            {"Camera1.fx", FormatDouble(camera_info.k[0])},
            {"Camera1.fy", FormatDouble(camera_info.k[4])},
            {"Camera1.cx", FormatDouble(camera_info.k[2])},
            {"Camera1.cy", FormatDouble(camera_info.k[5])},
            {"Camera.width", std::to_string(rgb_image.width)},
            {"Camera.height", std::to_string(rgb_image.height)},
            {"Camera.RGB", rgb_is_rgb ? "1" : "0"},
        };

        return RewriteSettingsFile(settings_file_, runtime_settings_file, overrides);
    }

    bool RGBDNode::ValidateCameraInfo(
        const sensor_msgs::msg::CameraInfo& camera_info,
        const sensor_msgs::msg::Image& rgb_image
    ) const {
        if (camera_info.width == 0 || camera_info.height == 0) {
            RCLCPP_ERROR(this->get_logger(), "CameraInfo has zero resolution");
            return false;
        }

        if (camera_info.width != rgb_image.width || camera_info.height != rgb_image.height) {
            RCLCPP_ERROR(
                this->get_logger(),
                "CameraInfo resolution (%ux%u) does not match RGB image (%ux%u)",
                camera_info.width,
                camera_info.height,
                rgb_image.width,
                rgb_image.height
            );
            return false;
        }

        if (camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0 || !std::isfinite(camera_info.k[0]) ||
            !std::isfinite(camera_info.k[4]) || !std::isfinite(camera_info.k[2]) ||
            !std::isfinite(camera_info.k[5])) {
            RCLCPP_ERROR(this->get_logger(), "CameraInfo contains invalid intrinsic values");
            return false;
        }

        return true;
    }

    bool RGBDNode::IsMonotonic(const SyncedFrame& frame) const noexcept {
        const std::int64_t rgb_stamp_ns = ToNanoseconds(frame.rgb_stamp);
        if (frame.sequence_id <= last_processed_sequence_) {
            return false;
        }

        if (rgb_stamp_ns <= last_processed_rgb_stamp_ns_) {
            return false;
        }

        return true;
    }

    void RGBDNode::LogDiagnostics() {
        if (!diagnostic_logging_) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (diagnostics_last_report_time_.time_since_epoch().count() == 0) {
            diagnostics_last_report_time_ = now;
            diagnostics_last_processed_total_ = processed_frames_total_.load();
            diagnostics_last_sync_delay_sum_ns_ = sync_delay_sum_ns_.load();
            return;
        }

        const auto elapsed = std::chrono::duration<double>(now - diagnostics_last_report_time_).count();
        const auto processed_total = processed_frames_total_.load();
        const auto sync_total = synced_pairs_total_.load();
        const auto sync_drop_total = dropped_sync_total_.load();
        const auto queue_drop_total = dropped_queue_total_.load();
        const auto duplicate_drop_total = dropped_duplicate_total_.load();
        const auto monotonic_sequence_drop_total = monotonic_sequence_drop_total_.load();
        const auto monotonic_stamp_drop_total = monotonic_stamp_drop_total_.load();
        const auto init_attempt_total = initialization_attempt_total_.load();
        const auto init_failure_total = initialization_failure_total_.load();
        const auto sync_delay_total_ns = sync_delay_sum_ns_.load();
        const auto latest_rgb_stamp_ns = latest_rgb_stamp_ns_.load();
        const auto latest_depth_stamp_ns = latest_depth_stamp_ns_.load();
        const auto latest_sync_delta_ns = latest_sync_delta_ns_.load();
        const auto max_sync_delta_ns = max_sync_delta_ns_.load();
        const auto camera_info_total = camera_info_total_.load();
        const auto latest_camera_info_stamp_ns = latest_camera_info_stamp_ns_.load();
        const std::string latest_rgb_stamp_text =
            latest_rgb_stamp_ns >= 0 ? FormatStampNs(latest_rgb_stamp_ns) : std::string("n/a");
        const std::string latest_depth_stamp_text =
            latest_depth_stamp_ns >= 0 ? FormatStampNs(latest_depth_stamp_ns) : std::string("n/a");
        const std::string latest_camera_info_stamp_text =
            latest_camera_info_stamp_ns >= 0 ? FormatStampNs(latest_camera_info_stamp_ns) : std::string("n/a");

        const auto processed_delta = processed_total - diagnostics_last_processed_total_;
        const double fps = elapsed > 0.0 ? static_cast<double>(processed_delta) / elapsed : 0.0;
        const double average_sync_delay_ms = sync_total > 0
            ? static_cast<double>(sync_delay_total_ns) / static_cast<double>(sync_total) / 1e6
            : 0.0;
        const std::size_t queue_depth = frame_queue_ ? frame_queue_->Size() : 0U;

        RCLCPP_INFO(
            this->get_logger(),
            "RGBD stats | fps=%.2f | queue=%zu/%zu | sync_avg=%.2f ms | sync_total=%llu | sync_drop=%llu | queue_drop=%llu | dup_drop=%llu | dup_seq=%llu | dup_stamp=%llu | rgb=%s | depth=%s | sync_last=%.3f ms | sync_max=%.3f ms | camera_info=%llu | camera_info_stamp=%s | init=%s | init_attempt=%llu | init_fail=%llu",
            fps,
            queue_depth,
            frame_queue_ ? frame_queue_->Capacity() : 0U,
            average_sync_delay_ms,
            static_cast<unsigned long long>(sync_total),
            static_cast<unsigned long long>(sync_drop_total),
            static_cast<unsigned long long>(queue_drop_total),
            static_cast<unsigned long long>(duplicate_drop_total),
            static_cast<unsigned long long>(monotonic_sequence_drop_total),
            static_cast<unsigned long long>(monotonic_stamp_drop_total),
            latest_rgb_stamp_text.c_str(),
            latest_depth_stamp_text.c_str(),
            ToMilliseconds(latest_sync_delta_ns),
            ToMilliseconds(max_sync_delta_ns),
            static_cast<unsigned long long>(camera_info_total),
            latest_camera_info_stamp_text.c_str(),
            slam_initialized_.load() ? "ready" : "waiting",
            static_cast<unsigned long long>(init_attempt_total),
            static_cast<unsigned long long>(init_failure_total)
        );

        diagnostics_last_report_time_ = now;
        diagnostics_last_processed_total_ = processed_total;
        diagnostics_last_sync_delay_sum_ns_ = sync_delay_total_ns;
    }

    void RGBDNode::PublishCachedMapPoints() {
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

    bool RGBDNode::IsRgbEncoding(const std::string& encoding) noexcept {
        const std::string lower = ToLowerCopy(encoding);
        return lower.find("rgb") != std::string::npos && lower.find("bgr") == std::string::npos;
    }

    std::string RGBDNode::FormatDouble(double value) {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(9) << value;
        return stream.str();
    }

}
